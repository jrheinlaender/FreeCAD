/***************************************************************************
 *   Copyright (c) 2010 Juergen Riegel <FreeCAD@juergen-riegel.net>        *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
# include <BRepBuilderAPI_MakeFace.hxx>
# include <BRepAdaptor_Surface.hxx>
# include <BRepAdaptor_Curve.hxx>
# include <BRepExtrema_DistShapeShape.hxx>
# include <BRep_Tool.hxx>
# include <BRepProj_Projection.hxx>
# include <Geom_Plane.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Compound.hxx>
# include <TopoDS_Face.hxx>
# include <TopExp_Explorer.hxx>
# include <gp_Pln.hxx>
# include <ShapeAnalysis.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
# include <TopExp.hxx>
# include <Standard_Version.hxx>
# include <GProp_GProps.hxx>
# include <BRepGProp.hxx>
#endif

#include <App/Plane.h>
#include <Base/Exception.h>
#include <Base/Parameter.h>
#include <App/Application.h>
#include <Mod/Part/App/modelRefine.h>
#include "FeatureSketchBased.h"
#include "DatumPlane.h"
#include "DatumLine.h"

using namespace PartDesign;

PROPERTY_SOURCE(PartDesign::SketchBased, PartDesign::Feature)

SketchBased::SketchBased()
{
    ADD_PROPERTY(Sketch,(0));
    ADD_PROPERTY_TYPE(Midplane,(0),"SketchBased", App::Prop_None, "Extrude symmetric to sketch face");
    ADD_PROPERTY_TYPE(Reversed, (0),"SketchBased", App::Prop_None, "Reverse extrusion direction");
    ADD_PROPERTY_TYPE(UpToFace,(0),"SketchBased",(App::PropertyType)(App::Prop_None),"Face where feature will end");
}

short SketchBased::mustExecute() const
{
    if (Sketch.isTouched() ||
        Midplane.isTouched() ||
        Reversed.isTouched() ||
        UpToFace.isTouched())
        return 1;
    return PartDesign::Feature::mustExecute();
}

void SketchBased::positionBySketch(void)
{
    Part::Part2DObject *sketch = static_cast<Part::Part2DObject*>(Sketch.getValue());
    if (sketch && sketch->getTypeId().isDerivedFrom(Part::Part2DObject::getClassTypeId())) {
        App::DocumentObject* support = sketch->Support.getValue();
        if (support == NULL)
            throw Base::Exception("Sketch with NULL support");
        if (support->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
            Part::Feature *part = static_cast<Part::Feature*>(support);
            this->Placement.setValue(part->Placement.getValue());
        } else if (support->getTypeId().isDerivedFrom(App::Plane::getClassTypeId())) {
            App::Plane *plane = static_cast<App::Plane*>(support);
            this->Placement.setValue(plane->Placement.getValue());
        } else {
            this->Placement.setValue(sketch->Placement.getValue());
        }
    }
}

void SketchBased::transformPlacement(const Base::Placement &transform)
{
    Part::Part2DObject *sketch = static_cast<Part::Part2DObject*>(Sketch.getValue());
    if (sketch && sketch->getTypeId().isDerivedFrom(Part::Part2DObject::getClassTypeId())) {
        Part::Feature *part = static_cast<Part::Feature*>(sketch->Support.getValue());
        if (part && part->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
            part->transformPlacement(transform);
        else
            sketch->transformPlacement(transform);
        positionBySketch();
    }
}

Part::Part2DObject* SketchBased::getVerifiedSketch() const {
    App::DocumentObject* result = Sketch.getValue();
    if (!result)
        throw Base::Exception("No sketch linked");
    if (!result->getTypeId().isDerivedFrom(Part::Part2DObject::getClassTypeId()))
        throw Base::Exception("Linked object is not a Sketch or Part2DObject");
    return static_cast<Part::Part2DObject*>(result);
}

// TODO: This code is taken from and duplicates code in Part2DObject::positionBySupport()
// Note: We cannot return a reference, because it will become Null.
// Not clear where, because we check for IsNull() here, but as soon as it is passed out of
// this method, it becomes null!
const TopoDS_Face SketchBased::getSupportFace() const {
    const App::PropertyLinkSub& Support = static_cast<Part::Part2DObject*>(Sketch.getValue())->Support;
    App::DocumentObject* ref = Support.getValue();

    if (ref->getTypeId().isDerivedFrom(App::Plane::getClassTypeId())) {
        TopoDS_Shape plane = Feature::makeShapeFromPlane(ref);
        return TopoDS::Face(plane);
    } else if (ref->getTypeId().isDerivedFrom(PartDesign::Plane::getClassTypeId())) {
        PartDesign::Plane* plane = static_cast<PartDesign::Plane*>(ref);
        return TopoDS::Face(plane->getShape());
    }

    Part::Feature *part = static_cast<Part::Feature*>(Support.getValue());
    if (!part || !part->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
        throw Base::Exception("No support in sketch");

    const std::vector<std::string> &sub = Support.getSubValues();
    assert(sub.size()==1);
    // get the selected sub shape (a Face)
    const Part::TopoShape &shape = part->Shape.getShape();
    if (shape._Shape.IsNull())
        throw Base::Exception("Sketch support shape is empty!");

    TopoDS_Shape sh = shape.getSubShape(sub[0].c_str());
    if (sh.IsNull())
        throw Base::Exception("Null shape in SketchBased::getSupportFace()!");

    const TopoDS_Face face = TopoDS::Face(sh);
    if (face.IsNull())
        throw Base::Exception("Null face in SketchBased::getSupportFace()!");

    BRepAdaptor_Surface adapt(face);
    if (adapt.GetType() != GeomAbs_Plane)
        throw Base::Exception("No planar face in SketchBased::getSupportFace()!");

    return face;
}

const TopoDS_Shape& SketchBased::getSupportShape() const {
    if (!Sketch.getValue())
        throw Base::Exception("No Sketch!");

    App::DocumentObject* SupportLink = static_cast<Part::Part2DObject*>(Sketch.getValue())->Support.getValue();
    Part::Feature* SupportObject = NULL;
    if (SupportLink && SupportLink->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
        SupportObject = static_cast<Part::Feature*>(SupportLink);

    if (SupportObject == NULL)
        throw Base::Exception("No support in Sketch!");

    const TopoDS_Shape& result = SupportObject->Shape.getValue();
    if (result.IsNull())
        throw Base::Exception("Support shape is invalid");
    TopExp_Explorer xp (result, TopAbs_SOLID);
    if (!xp.More())
        throw Base::Exception("Support shape is not a solid");

    return result;
}

const Part::TopoShape& SketchBased::getSupportTopoShape() const {
    if (!Sketch.getValue())
        throw Base::Exception("No Sketch!");

    App::DocumentObject* SupportLink = static_cast<Part::Part2DObject*>(Sketch.getValue())->Support.getValue();
    Part::Feature* SupportObject = NULL;
    if (SupportLink && SupportLink->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
        SupportObject = static_cast<Part::Feature*>(SupportLink);

    if (SupportObject == NULL)
        throw Base::Exception("No support in Sketch!");

    const Part::TopoShape& result = SupportObject->Shape.getShape();
    if (result._Shape.IsNull())
        throw Base::Exception("Support feature's TopoShape is invalid");

    return result;
}

int SketchBased::getSketchAxisCount(void) const
{
    Part::Part2DObject *sketch = static_cast<Part::Part2DObject*>(Sketch.getValue());
    return sketch->getAxisCount();
}

void SketchBased::onChanged(const App::Property* prop)
{
    if (prop == &Sketch) {
        // if attached to a sketch then mark it as read-only
        this->Placement.StatusBits.set(2, Sketch.getValue() != 0);
    }

    Feature::onChanged(prop);
}

void SketchBased::getUpToFaceFromLinkSub(TopoDS_Face& upToFace,
                                         const App::PropertyLinkSub& refFace)
{
    App::DocumentObject* ref = refFace.getValue();
    std::vector<std::string> subStrings = refFace.getSubValues();

    if (ref == NULL)
        throw Base::Exception("SketchBased: Up to face: No face selected");

    if (ref->getTypeId().isDerivedFrom(App::Plane::getClassTypeId())) {
        upToFace = TopoDS::Face(makeShapeFromPlane(ref));
        return;
    } else if (ref->getTypeId().isDerivedFrom(PartDesign::Plane::getClassTypeId())) {
        Part::Datum* datum = static_cast<Part::Datum*>(ref);
        upToFace = TopoDS::Face(datum->getShape());
        return;
    }

    if (!ref->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
        throw Base::Exception("SketchBased: Up to face: Must be face of a feature");
    Part::TopoShape baseShape = static_cast<Part::Feature*>(ref)->Shape.getShape();

    if (subStrings.empty() || subStrings[0].empty())
        throw Base::Exception("SketchBased: Up to face: No face selected");
    // TODO: Check for multiple UpToFaces?

    upToFace = TopoDS::Face(baseShape.getSubShape(subStrings[0].c_str()));
    if (upToFace.IsNull())
        throw Base::Exception("SketchBased: Up to face: Failed to extract face");
}

void SketchBased::getUpToFace(TopoDS_Face& upToFace,
                              const TopoDS_Shape& support,
                              const TopoDS_Face& supportface,
                              const TopoDS_Shape& sketchshape,
                              const std::string& method,
                              const gp_Dir& dir,
                              const double offset)
{
    if ((method == "UpToLast") || (method == "UpToFirst")) {
        // Check for valid support object
        if (support.IsNull())
            throw Base::Exception("SketchBased: Up to face: No support in Sketch and no base feature!");

        std::vector<Part::cutFaces> cfaces = Part::findAllFacesCutBy(support, sketchshape, dir);
        if (cfaces.empty())
            throw Base::Exception("SketchBased: Up to face: No faces found in this direction");

        // Find nearest/furthest face
        std::vector<Part::cutFaces>::const_iterator it, it_near, it_far;
        it_near = it_far = cfaces.begin();
        for (it = cfaces.begin(); it != cfaces.end(); it++)
            if (it->distsq > it_far->distsq)
                it_far = it;
            else if (it->distsq < it_near->distsq)
                it_near = it;
        upToFace = (method == "UpToLast" ? it_far->face : it_near->face);
    }

    // Check whether the face has limits or not. Unlimited faces have no wire
    // Note: Datum planes are always unlimited
    TopExp_Explorer Ex(upToFace,TopAbs_WIRE);
    if (Ex.More()) {
        // Remove the limits of the upToFace so that the extrusion works even if sketchshape is larger
        // than the upToFace
        bool remove_limits = false;
        for (Ex.Init(sketchshape,TopAbs_FACE); Ex.More(); Ex.Next()) {
            // Get outermost wire of sketch face
            TopoDS_Face sketchface = TopoDS::Face(Ex.Current());
            TopoDS_Wire outerWire = ShapeAnalysis::OuterWire(sketchface);
            if (!checkWireInsideFace(outerWire, upToFace, dir)) {
                remove_limits = true;
                break;
            }
        }

        if (remove_limits) {
            // Note: Using an unlimited face every time gives unnecessary failures for concave faces
            TopLoc_Location loc = upToFace.Location();
            BRepAdaptor_Surface adapt(upToFace, Standard_False);
            BRepBuilderAPI_MakeFace mkFace(adapt.Surface().Surface()
    #if OCC_VERSION_HEX >= 0x060502
                  , Precision::Confusion()
    #endif
            );
            if (!mkFace.IsDone())
                throw Base::Exception("SketchBased: Up To Face: Failed to create unlimited face");
            upToFace = TopoDS::Face(mkFace.Shape());
            upToFace.Location(loc);
        }
    }

    // Check that the upToFace does not intersect the sketch face and
    // is not parallel to the extrusion direction (for simplicity, supportface is used instead of sketchshape)
    BRepAdaptor_Surface adapt1(TopoDS::Face(supportface));
    BRepAdaptor_Surface adapt2(TopoDS::Face(upToFace));

    if (adapt2.GetType() == GeomAbs_Plane) {
        if (adapt1.Plane().Axis().IsNormal(adapt2.Plane().Axis(), Precision::Confusion()))
            throw Base::Exception("SketchBased: Up to face: Must not be parallel to extrusion direction!");
    }

    // We must measure from sketchshape, not supportface, here
    BRepExtrema_DistShapeShape distSS(sketchshape, upToFace);
    if (distSS.Value() < Precision::Confusion())
        throw Base::Exception("SketchBased: Up to face: Must not intersect sketch!");

    // Move the face in the extrusion direction
    // TODO: For non-planar faces, we could consider offsetting the surface
    if (fabs(offset) > Precision::Confusion()) {
        if (adapt2.GetType() == GeomAbs_Plane) {
            gp_Trsf mov;
            mov.SetTranslation(offset * gp_Vec(dir));
            TopLoc_Location loc(mov);
            upToFace.Move(loc);
        } else {
            throw Base::Exception("SketchBased: Up to Face: Offset not supported yet for non-planar faces");
        }
    }
}

const bool SketchBased::checkWireInsideFace(const TopoDS_Wire& wire, const TopoDS_Face& face,
                                            const gp_Dir& dir) {
    // Project wire onto the face (face, not surface! So limits of face apply)
    // FIXME: The results of BRepProj_Projection do not seem to be very stable. Sometimes they return no result
    // even in the simplest projection case.
    // FIXME: Checking for Closed() is wrong because this has nothing to do with the wire itself being closed
    // But ShapeAnalysis_Wire::CheckClosed() doesn't give correct results either.
    BRepProj_Projection proj(wire, face, dir);
    return (proj.More() && proj.Current().Closed());
}

void SketchBased::remapSupportShape(const TopoDS_Shape& newShape)
{
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(newShape, TopAbs_FACE, faceMap);

    // here we must reset the placement otherwise the geometric matching doesn't work
    Part::TopoShape shape = this->Shape.getValue();
    shape._Shape.Location(TopLoc_Location());

    std::vector<App::DocumentObject*> refs = this->getInList();
    for (std::vector<App::DocumentObject*>::iterator it = refs.begin(); it != refs.end(); ++it) {
        std::vector<App::Property*> props;
        (*it)->getPropertyList(props);
        for (std::vector<App::Property*>::iterator jt = props.begin(); jt != props.end(); ++jt) {
            if (!(*jt)->isDerivedFrom(App::PropertyLinkSub::getClassTypeId()))
                continue;
            App::PropertyLinkSub* link = static_cast<App::PropertyLinkSub*>(*jt);
            if (link->getValue() != this)
                continue;
            std::vector<std::string> subValues = link->getSubValues();
            std::vector<std::string> newSubValues;

            for (std::vector<std::string>::iterator it = subValues.begin(); it != subValues.end(); ++it) {
                std::string shapetype;
                if (it->size() > 4 && it->substr(0,4) == "Face") {
                    shapetype = "Face";
                }
                else if (it->size() > 4 && it->substr(0,4) == "Edge") {
                    shapetype = "Edge";
                }
                else if (it->size() > 6 && it->substr(0,6) == "Vertex") {
                    shapetype = "Vertex";
                }
                else {
                    newSubValues.push_back(*it);
                    continue;
                }

                bool success = false;
                TopoDS_Shape element;
                try {
                    element = shape.getSubShape(it->c_str());
                }
                catch (Standard_Failure) {
                    // This shape doesn't even exist, so no chance to do some tests
                    newSubValues.push_back(*it);
                    continue;
                }
                try {
                    // as very first test check if old face and new face are parallel planes
                    TopoDS_Shape newElement = Part::TopoShape(newShape).getSubShape(it->c_str());
                    if (isParallelPlane(element, newElement)) {
                        newSubValues.push_back(*it);
                        success = true;
                    }
                }
                catch (Standard_Failure) {
                }
                // try an exact matching
                if (!success) {
                    for (int i=1; i<faceMap.Extent(); i++) {
                        if (isQuasiEqual(element, faceMap.FindKey(i))) {
                            std::stringstream str;
                            str << shapetype << i;
                            newSubValues.push_back(str.str());
                            success = true;
                            break;
                        }
                    }
                }
                // if an exact matching fails then try to compare only the geometries
                if (!success) {
                    for (int i=1; i<faceMap.Extent(); i++) {
                        if (isEqualGeometry(element, faceMap.FindKey(i))) {
                            std::stringstream str;
                            str << shapetype << i;
                            newSubValues.push_back(str.str());
                            success = true;
                            break;
                        }
                    }
                }

                // the new shape couldn't be found so keep the old sub-name
                if (!success)
                    newSubValues.push_back(*it);
            }

            link->setValue(this, newSubValues);
        }
    }
}

namespace PartDesign {
struct gp_Pnt_Less  : public std::binary_function<const gp_Pnt&,
                                                  const gp_Pnt&, bool>
{
    bool operator()(const gp_Pnt& p1,
                    const gp_Pnt& p2) const
    {
        if (fabs(p1.X() - p2.X()) > Precision::Confusion())
            return p1.X() < p2.X();
        if (fabs(p1.Y() - p2.Y()) > Precision::Confusion())
            return p1.Y() < p2.Y();
        if (fabs(p1.Z() - p2.Z()) > Precision::Confusion())
            return p1.Z() < p2.Z();
        return false; // points are considered to be equal
    }
};
}

bool SketchBased::isQuasiEqual(const TopoDS_Shape& s1, const TopoDS_Shape& s2) const
{
    if (s1.ShapeType() != s2.ShapeType())
        return false;
    TopTools_IndexedMapOfShape map1, map2;
    TopExp::MapShapes(s1, TopAbs_VERTEX, map1);
    TopExp::MapShapes(s2, TopAbs_VERTEX, map2);
    if (map1.Extent() != map2.Extent())
        return false;

    std::vector<gp_Pnt> p1;
    for (int i=1; i<=map1.Extent(); i++) {
        const TopoDS_Vertex& v = TopoDS::Vertex(map1.FindKey(i));
        p1.push_back(BRep_Tool::Pnt(v));
    }
    std::vector<gp_Pnt> p2;
    for (int i=1; i<=map2.Extent(); i++) {
        const TopoDS_Vertex& v = TopoDS::Vertex(map2.FindKey(i));
        p2.push_back(BRep_Tool::Pnt(v));
    }

    std::sort(p1.begin(), p1.end(), gp_Pnt_Less());
    std::sort(p2.begin(), p2.end(), gp_Pnt_Less());

    if (p1.size() != p2.size())
        return false;

    std::vector<gp_Pnt>::iterator it = p1.begin(), jt = p2.begin();
    for (; it != p1.end(); ++it, ++jt) {
        if (!(*it).IsEqual(*jt, Precision::Confusion()))
            return false;
    }

    return true;
}

bool SketchBased::isEqualGeometry(const TopoDS_Shape& s1, const TopoDS_Shape& s2) const
{
    if (s1.ShapeType() == TopAbs_FACE && s2.ShapeType() == TopAbs_FACE) {
        BRepAdaptor_Surface a1(TopoDS::Face(s1));
        BRepAdaptor_Surface a2(TopoDS::Face(s2));
        if (a1.GetType() == GeomAbs_Plane && a2.GetType() == GeomAbs_Plane) {
            gp_Pln p1 = a1.Plane();
            gp_Pln p2 = a2.Plane();
            if (p1.Distance(p2.Location()) < Precision::Confusion()) {
                const gp_Dir& d1 = p1.Axis().Direction();
                const gp_Dir& d2 = p2.Axis().Direction();
                if (d1.IsParallel(d2, Precision::Confusion()))
                    return true;
            }
        }
    }
    else if (s1.ShapeType() == TopAbs_EDGE && s2.ShapeType() == TopAbs_EDGE) {
    }
    else if (s1.ShapeType() == TopAbs_VERTEX && s2.ShapeType() == TopAbs_VERTEX) {
        gp_Pnt p1 = BRep_Tool::Pnt(TopoDS::Vertex(s1));
        gp_Pnt p2 = BRep_Tool::Pnt(TopoDS::Vertex(s2));
        return p1.Distance(p2) < Precision::Confusion();
    }

    return false;
}

bool SketchBased::isParallelPlane(const TopoDS_Shape& s1, const TopoDS_Shape& s2) const
{
    if (s1.ShapeType() == TopAbs_FACE && s2.ShapeType() == TopAbs_FACE) {
        BRepAdaptor_Surface a1(TopoDS::Face(s1));
        BRepAdaptor_Surface a2(TopoDS::Face(s2));
        if (a1.GetType() == GeomAbs_Plane && a2.GetType() == GeomAbs_Plane) {
            gp_Pln p1 = a1.Plane();
            gp_Pln p2 = a2.Plane();
            const gp_Dir& d1 = p1.Axis().Direction();
            const gp_Dir& d2 = p2.Axis().Direction();
            if (d1.IsParallel(d2, Precision::Confusion()))
                return true;
        }
    }

    return false;
}

TopoDS_Shape SketchBased::refineShapeIfActive(const TopoDS_Shape& oldShape) const
{
    Base::Reference<ParameterGrp> hGrp = App::GetApplication().GetUserParameter()
        .GetGroup("BaseApp")->GetGroup("Preferences")->GetGroup("Mod/PartDesign");
    if (hGrp->GetBool("RefineModel", false)) {
        Part::BRepBuilderAPI_RefineModel mkRefine(oldShape);
        TopoDS_Shape resShape = mkRefine.Shape();
        return resShape;
    }

    return oldShape;
}


bool SketchBased::isSupportDatum() const
{
    if (!Sketch.getValue())
        return 0;
    App::DocumentObject* SupportObject = static_cast<Part::Part2DObject*>(Sketch.getValue())->Support.getValue();
    if (SupportObject == NULL)
        throw Base::Exception("No support in Sketch!");

    return isDatum(SupportObject);
}
const double SketchBased::getReversedAngle(const Base::Vector3d &b, const Base::Vector3d &v)
{
    try {
        Part::Part2DObject* sketch = getVerifiedSketch();        
        Part::TopoShape theSketch = sketch->Shape.getShape();
        theSketch.makeFace();
        TopoDS_Shape sketchshape = theSketch._Shape;

        // get centre of gravity of the sketch face
        GProp_GProps props;
        BRepGProp::SurfaceProperties(sketchshape, props);
        gp_Pnt cog = props.CentreOfMass();
        Base::Vector3d p_cog(cog.X(), cog.Y(), cog.Z());
        // get direction to cog from its projection on the revolve axis
        Base::Vector3d perp_dir = p_cog - p_cog.Perpendicular(b, v);
        // get cross product of projection direction with revolve axis direction
        Base::Vector3d cross = v % perp_dir;
        // get sketch vector pointing away from support material
        Base::Placement SketchPos = sketch->Placement.getValue();
        Base::Rotation SketchOrientation = SketchPos.getRotation();
        Base::Vector3d SketchNormal(0,0,1);
        SketchOrientation.multVec(SketchNormal,SketchNormal);

        return SketchNormal * cross;
    }
    catch (...) {
        return Reversed.getValue();
    }
}

void SketchBased::getAxis(const App::DocumentObject *pcReferenceAxis, const std::vector<std::string> &subReferenceAxis,
                          Base::Vector3d& base, Base::Vector3d& dir)
{
    dir = Base::Vector3d(0,0,0); // If unchanged signals that no valid axis was found
    if (pcReferenceAxis == NULL)
        return;

    Part::Part2DObject* sketch = getVerifiedSketch();
    Base::Placement SketchPlm = sketch->Placement.getValue();
    Base::Vector3d SketchPos = SketchPlm.getPosition();
    Base::Rotation SketchOrientation = SketchPlm.getRotation();
    Base::Vector3d SketchVector(0,0,1);
    SketchOrientation.multVec(SketchVector,SketchVector);
    gp_Pln sketchplane(gp_Pnt(SketchPos.x, SketchPos.y, SketchPos.z), gp_Dir(SketchVector.x, SketchVector.y, SketchVector.z));

    // get reference axis
    if (pcReferenceAxis->getTypeId().isDerivedFrom(PartDesign::Line::getClassTypeId())) {
        const PartDesign::Line* line = static_cast<const PartDesign::Line*>(pcReferenceAxis);
        base = line->getBasePoint();
        dir = line->getDirection();

        // Check that axis is co-planar with sketch plane!
        if (!sketchplane.Contains(gp_Lin(gp_Pnt(base.x, base.y, base.z), gp_Dir(dir.x, dir.y, dir.z)), Precision::Confusion(), Precision::Confusion()))
            throw Base::Exception("Rotation axis must be coplanar with the sketch plane");
    } else if (pcReferenceAxis->getTypeId().isDerivedFrom(PartDesign::Feature::getClassTypeId())) {
        if (subReferenceAxis[0].empty())
            throw Base::Exception("No rotation axis reference specified");
        const Part::Feature* refFeature = static_cast<const Part::Feature*>(pcReferenceAxis);
        Part::TopoShape refShape = refFeature->Shape.getShape();
        TopoDS_Shape ref = refShape.getSubShape(subReferenceAxis[0].c_str());

        if (ref.ShapeType() == TopAbs_EDGE) {
            TopoDS_Edge refEdge = TopoDS::Edge(ref);
            if (refEdge.IsNull())
                throw Base::Exception("Failed to extract rotation edge");
            BRepAdaptor_Curve adapt(refEdge);
            if (adapt.GetType() != GeomAbs_Line)
                throw Base::Exception("Rotation edge must be a straight line");

            gp_Pnt b = adapt.Line().Location();
            base = Base::Vector3d(b.X(), b.Y(), b.Z());
            gp_Dir d = adapt.Line().Direction();
            dir = Base::Vector3d(d.X(), d.Y(), d.Z());
            // Check that axis is co-planar with sketch plane!
            if (!sketchplane.Contains(adapt.Line(), Precision::Confusion(), Precision::Confusion()))
                throw Base::Exception("Rotation axis must be coplanar with the sketch plane");
        } else {
            throw Base::Exception("Rotation reference must be an edge");
        }
    } else if (pcReferenceAxis && pcReferenceAxis == sketch) {
        bool hasValidAxis=false;
        Base::Axis axis;
        if (subReferenceAxis[0] == "V_Axis") {
            hasValidAxis = true;
            axis = sketch->getAxis(Part::Part2DObject::V_Axis);
        }
        else if (subReferenceAxis[0] == "H_Axis") {
            hasValidAxis = true;
            axis = sketch->getAxis(Part::Part2DObject::H_Axis);
        }
        else if (subReferenceAxis[0].size() > 4 && subReferenceAxis[0].substr(0,4) == "Axis") {
            int AxId = std::atoi(subReferenceAxis[0].substr(4,4000).c_str());
            if (AxId >= 0 && AxId < sketch->getAxisCount()) {
                hasValidAxis = true;
                axis = sketch->getAxis(AxId);
            }
        }
        if (hasValidAxis) {
            axis *= SketchPlm;
            base=axis.getBase();
            dir=axis.getDirection();
        }
    }
}
