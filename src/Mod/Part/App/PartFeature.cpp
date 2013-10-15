/***************************************************************************
 *   Copyright (c) Jürgen Riegel          (juergen.riegel@web.de) 2002     *
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
# include <gp_Trsf.hxx>
# include <gp_Ax1.hxx>
# include <BRepBuilderAPI_MakeShape.hxx>
# include <BRepPrimAPI_MakePrism.hxx>
# include <BRepPrimAPI_MakeRevol.hxx>
# include <BRepFeat_MakePrism.hxx>
# include <BRepAlgoAPI_Fuse.hxx>
# include <BRepAlgoAPI_Common.hxx>
# include <TopTools_ListIteratorOfListOfShape.hxx>
# include <TopExp.hxx>
# include <TopoDS.hxx>
# include <BRepAdaptor_Curve.hxx>
# include <BRepAdaptor_Surface.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <Standard_Failure.hxx>
# include <TopoDS_Shape.hxx>
# include <TopoDS_Face.hxx>
# include <gp_Dir.hxx>
# include <gp_Pln.hxx> // for Precision::Confusion()
# include <gp_Circ.hxx>
# include <Bnd_Box.hxx>
# include <BRepBndLib.hxx>
#endif


#include <strstream>
#include <Base/Console.h>
#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>
#include <Base/FileInfo.h>
#include <Base/Stream.h>
#include <Base/Placement.h>
#include <Base/Rotation.h>
#include <App/FeaturePythonPyImp.h>

#include "PartFeature.h"
#include "PartFeaturePy.h"

using namespace Part;


PROPERTY_SOURCE(Part::Feature, App::GeoFeature)


Feature::Feature(void) 
{
    ADD_PROPERTY(Shape, (TopoDS_Shape()));
}

Feature::~Feature()
{
}

short Feature::mustExecute(void) const
{
    return GeoFeature::mustExecute();
}

App::DocumentObjectExecReturn *Feature::recompute(void)
{
    try {
        return App::GeoFeature::recompute();
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        App::DocumentObjectExecReturn* ret = new App::DocumentObjectExecReturn(e->GetMessageString());
        if (ret->Why.empty()) ret->Why = "Unknown OCC exception";
        return ret;
    }
}

App::DocumentObjectExecReturn *Feature::execute(void)
{
    return App::DocumentObject::StdReturn;
}

PyObject *Feature::getPyObject(void)
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new PartFeaturePy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}

std::vector<PyObject *> Feature::getPySubObjects(const std::vector<std::string>& NameVec) const
{
    std::vector<PyObject *> temp;
    for(std::vector<std::string>::const_iterator it=NameVec.begin();it!=NameVec.end();++it){
        PyObject *obj = Shape.getShape().getPySubShape((*it).c_str());
        if(obj)
            temp.push_back(obj);
    }
    return temp;
}

void Feature::onChanged(const App::Property* prop)
{
    // if the placement has changed apply the change to the point data as well
    if (prop == &this->Placement) {
        TopoShape& shape = const_cast<TopoShape&>(this->Shape.getShape());
        shape.setTransform(this->Placement.getValue().toMatrix());
    }
    // if the point data has changed check and adjust the transformation as well
    else if (prop == &this->Shape) {
        if (this->isRecomputing()) {
            TopoShape& shape = const_cast<TopoShape&>(this->Shape.getShape());
            shape.setTransform(this->Placement.getValue().toMatrix());
        }
        else {
            Base::Placement p;
            // shape must not be null to override the placement
            if (!this->Shape.getValue().IsNull()) {
                p.fromMatrix(this->Shape.getShape().getTransform());
                if (p != this->Placement.getValue())
                    this->Placement.setValue(p);
            }
        }
    }
    
    GeoFeature::onChanged(prop);
}

TopLoc_Location Feature::getLocation() const
{
    Base::Placement pl = this->Placement.getValue();
    Base::Rotation rot(pl.getRotation());
    Base::Vector3d axis;
    double angle;
    rot.getValue(axis, angle);
    gp_Trsf trf;
    trf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(axis.x, axis.y, axis.z)), angle);
    trf.SetTranslationPart(gp_Vec(pl.getPosition().x,pl.getPosition().y,pl.getPosition().z));
    return TopLoc_Location(trf);
}

ShapeHistory Feature::buildHistory(BRepBuilderAPI_MakeShape& mkShape, TopAbs_ShapeEnum type,
                                   const TopoDS_Shape& newS, const TopoDS_Shape& oldS)
{
    ShapeHistory history;
    history.type = type;

    TopTools_IndexedMapOfShape newM, oldM;
    TopExp::MapShapes(newS, type, newM); // map containing all old objects of type "type"
    TopExp::MapShapes(oldS, type, oldM); // map containing all new objects of type "type"

    // Look at all objects in the old shape and try to find the modified object in the new shape
    for (int i=1; i<=oldM.Extent(); i++) {
        bool found = false;
        TopTools_ListIteratorOfListOfShape it;
        // Find all new objects that are a modification of the old object (e.g. a face was resized)
        for (it.Initialize(mkShape.Modified(oldM(i))); it.More(); it.Next()) {
            found = true;
            for (int j=1; j<=newM.Extent(); j++) { // one old object might create several new ones!
                if (newM(j).IsPartner(it.Value())) {
                    history.shapeMap[i-1].push_back(j-1); // adjust indices to start at zero
                    break;
                }
            }
        }

        // Find all new objects that were generated from an old object (e.g. a face generated from an edge)
        for (it.Initialize(mkShape.Generated(oldM(i))); it.More(); it.Next()) {
            found = true;
            for (int j=1; j<=newM.Extent(); j++) {
                if (newM(j).IsPartner(it.Value())) {
                    history.shapeMap[i-1].push_back(j-1);
                    break;
                }
            }
        }

        if (!found) {
            // Find all old objects that don't exist any more (e.g. a face was completely cut away)
            if (mkShape.IsDeleted(oldM(i))) {
                history.shapeMap[i-1] = std::vector<int>();
            }
            else {
                // Mop up the rest (will this ever be reached?)
                for (int j=1; j<=newM.Extent(); j++) {
                    if (newM(j).IsPartner(oldM(i))) {
                        history.shapeMap[i-1].push_back(j-1);
                        break;
                    }
                }
            }
        }
    }

    return history;
}

ShapeHistory Feature::joinHistory(const ShapeHistory& oldH, const ShapeHistory& newH)
{
    ShapeHistory join;
    join.type = oldH.type;

    for (ShapeHistory::MapList::const_iterator it = oldH.shapeMap.begin(); it != oldH.shapeMap.end(); ++it) {
        int old_shape_index = it->first;
        if (it->second.empty())
            join.shapeMap[old_shape_index] = ShapeHistory::List();
        for (ShapeHistory::List::const_iterator jt = it->second.begin(); jt != it->second.end(); ++jt) {
            ShapeHistory::MapList::const_iterator kt = newH.shapeMap.find(*jt);
            if (kt != newH.shapeMap.end()) {
                ShapeHistory::List& ary = join.shapeMap[old_shape_index];
                ary.insert(ary.end(), kt->second.begin(), kt->second.end());
            }
        }
    }

    return join;
}

    /// returns the type name of the ViewProvider
const char* Feature::getViewProviderName(void) const {
    return "PartGui::ViewProviderPart";
}

// ---------------------------------------------------------

PROPERTY_SOURCE(Part::FilletBase, Part::Feature)

FilletBase::FilletBase()
{
    ADD_PROPERTY(Base,(0));
    ADD_PROPERTY(Edges,(0,0,0));
    Edges.setSize(0);
}

short FilletBase::mustExecute() const
{
    if (Base.isTouched() || Edges.isTouched())
        return 1;
    return 0;
}

// ---------------------------------------------------------

PROPERTY_SOURCE(Part::FeatureExt, Part::Feature)



namespace App {
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(Part::FeaturePython, Part::Feature)
template<> const char* Part::FeaturePython::getViewProviderName(void) const {
    return "PartGui::ViewProviderPython";
}
template<> PyObject* Part::FeaturePython::getPyObject(void) {
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new FeaturePythonPyT<Part::PartFeaturePy>(this),true);
    }
    return Py::new_reference_to(PythonObject);
}
/// @endcond

// explicit template instantiation
template class PartExport FeaturePythonT<Part::Feature>;
}

// ----------------------------------------------------------------

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <gce_MakeLin.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <IntCurveSurface_IntersectionPoint.hxx>
#include <gce_MakeDir.hxx>

std::vector<Part::cutFaces> Part::findAllFacesCutBy(
        const TopoDS_Shape& shape, const TopoDS_Shape& face, const gp_Dir& dir)
{
    // Find the centre of gravity of the face
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face,props);
    gp_Pnt cog = props.CentreOfMass();

    // create a line through the centre of gravity
    gp_Lin line = gce_MakeLin(cog, dir);

    // Find intersection of line with all faces of the shape
    std::vector<cutFaces> result;
    BRepIntCurveSurface_Inter mkSection;
    // TODO: Less precision than Confusion() should be OK?

    for (mkSection.Init(shape, line, Precision::Confusion()); mkSection.More(); mkSection.Next()) {
        gp_Pnt iPnt = mkSection.Pnt();
        double dsq = cog.SquareDistance(iPnt);

        if (dsq < Precision::Confusion())
            continue; // intersection with original face

        // Find out which side of the original face the intersection is on
        gce_MakeDir mkDir(cog, iPnt);
        if (!mkDir.IsDone())
            continue; // some error (appears highly unlikely to happen, though...)

        if (mkDir.Value().IsOpposite(dir, Precision::Confusion()))
            continue; // wrong side of face (opposite to extrusion direction)

        cutFaces newF;
        newF.face = mkSection.Face();
        newF.distsq = dsq;
        result.push_back(newF);
    }

    return result;
}

const bool Part::checkIntersection(const TopoDS_Shape& first, const TopoDS_Shape& second,
                                   const bool quick, const bool touch_is_intersection) {
    Bnd_Box first_bb, second_bb;
    BRepBndLib::Add(first, first_bb);
    first_bb.SetGap(0);
    BRepBndLib::Add(second, second_bb);
    second_bb.SetGap(0);

    // Note: This test fails if the objects are touching one another at zero distance
    if (first_bb.IsOut(second_bb))
        return false; // no intersection
    if (quick)
        return true; // assumed intersection

    // Try harder
    if (touch_is_intersection) {
        // If both shapes fuse to a single solid, then they intersect
        BRepAlgoAPI_Fuse mkFuse(first, second);
        if (!mkFuse.IsDone())
            return false;
        if (mkFuse.Shape().IsNull())
            return false;

        // Did we get one or two solids?
        TopExp_Explorer xp;
        xp.Init(mkFuse.Shape(),TopAbs_SOLID);
        if (xp.More()) {
            // At least one solid
            xp.Next();
            return (xp.More() == Standard_False);
        } else {
            return false;
        }
    } else {
        // If both shapes have common material, then they intersect
        BRepAlgoAPI_Common mkCommon(first, second);
        if (!mkCommon.IsDone())
            return false;
        if (mkCommon.Shape().IsNull())
            return false;

        // Did we get a solid?
        TopExp_Explorer xp;
        xp.Init(mkCommon.Shape(),TopAbs_SOLID);
        return (xp.More() == Standard_True);
    }
}

// ------------ Topological naming (simple but imperfect) ------------

const unsigned numShapeTypes = 3;
TopAbs_ShapeEnum shapeTypesData[] = {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX};
std::vector<TopAbs_ShapeEnum> shapeTypes(shapeTypesData, shapeTypesData + sizeof(shapeTypesData) / shapeTypesData[0]);

/// Find all faces, edges and vertices of the shape
std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> extractSubShapes(const TopoDS_Shape& shape)
{
    // Extract all subshapes from the shape
    // Note: If we don't use pointers here there is some compiler error
    // about TopTools_IndexedMapOfShape::TopTools_IndexedMapOfShape() being private!
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> result;
    TopTools_IndexedMapOfShape* M;
    for (int t = 0; t < numShapeTypes; ++t) {
        M = new TopTools_IndexedMapOfShape();
        TopExp::MapShapes(shape, shapeTypes[t], *M);
        result[shapeTypes[t]] = M;
    }

    return result;
}

/// Clear the data structure created by extractSubShapes()
void clearSubShapes(std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> map)
{
    for (int t = 0; t < numShapeTypes; t++)
        delete map[shapeTypes[t]];
}

ShapeRef::ShapeRef(const std::string& subName)
{
    if (subName.find("Face") != std::string::npos) {
        type = TopAbs_FACE;
        index = atoi(subName.substr(4).c_str()) - 1;
    } else if (subName.find("Edge") != std::string::npos) {
        type = TopAbs_EDGE;
        index = atoi(subName.substr(4).c_str()) - 1;
    } else if (subName.find("Vertex") != std::string::npos) {
        type = TopAbs_VERTEX;
        index = atoi(subName.substr(6).c_str()) - 1;
    } else {
        type = TopAbs_SHAPE;
    }
}

const std::string ShapeRef::toSubName() const
{
    std::stringstream strm;
    if (type == TopAbs_FACE)
        strm << "Face" << index+1;
    else if (type == TopAbs_EDGE)
        strm << "Edge" << index+1;
    else if (type == TopAbs_VERTEX)
        strm << "Vertex" << index+1;
    else if (type == TopAbs_SHAPE)
        return std::string("");

    return strm.str();
}

const bool ShapeRef::operator<(const ShapeRef& other) const
{
    if (type < other.type)
        return true;
    else {
        if (type == other.type) {
            if (index < other.index)
                return true;
        }
    }

    return false;
}

// Compare two shapes by their geometric properties
const bool compareShapes(const TopoDS_Shape& s1, const TopoDS_Shape& s2, const TopAbs_ShapeEnum type) {
    // Note: We rely on the type being correct, otherwise there will be a crash
    if (type == TopAbs_VERTEX) {
        TopoDS_Vertex v1 = TopoDS::Vertex(s1);
        TopoDS_Vertex v2 = TopoDS::Vertex(s2);
        gp_Pnt p1 = BRep_Tool::Pnt(v1);
        gp_Pnt p2 = BRep_Tool::Pnt(v2);
        return p1.IsEqual(p2, Precision::Confusion());
    } else if (type == TopAbs_EDGE) {
        TopoDS_Edge e1 = TopoDS::Edge(s1);
        TopoDS_Edge e2 = TopoDS::Edge(s2);
        BRepAdaptor_Curve c1(e1);
        BRepAdaptor_Curve c2(e2);

        // First check that curve types are identical
        GeomAbs_CurveType t1 = c1.GetType();
        GeomAbs_CurveType t2 = c2.GetType();
        if (t1 != t2)
            return false;

        // Compare start and end points of the edges
        gp_Pnt pf1 = BRep_Tool::Pnt(TopExp::FirstVertex(e1));
        gp_Pnt pl1 = BRep_Tool::Pnt(TopExp::LastVertex(e1));
        gp_Pnt pf2 = BRep_Tool::Pnt(TopExp::FirstVertex(e2));
        gp_Pnt pl2 = BRep_Tool::Pnt(TopExp::LastVertex(e2));
        if (!(pf1.IsEqual(pf2, Precision::Confusion()) && pl1.IsEqual(pl2, Precision::Confusion())))
            if (!(pf1.IsEqual(pl2, Precision::Confusion()) && pl1.IsEqual(pf2, Precision::Confusion())))
                return false;

        switch (t1) {
        case GeomAbs_Line:
        {
            return true; // Lines with same start and end points must be identical
        }
        case GeomAbs_Circle:
        {
            // This is sufficient for arcs and full circles because we already compared the start and end points of the edge
            return ((std::fabs(c1.Circle().Radius() - c2.Circle().Radius()) <= Precision::Confusion()) &&
                    (c1.Circle().Axis().IsCoaxial(c2.Circle().Axis(), Precision::Confusion(), Precision::Confusion())));
        }
        default:
        {
            throw Base::Exception("Comparing edges other than linear or circular is not implemented yet");
        }
        }
    } else if (type == TopAbs_FACE) {
        TopoDS_Face f1 = TopoDS::Face(s1);
        TopoDS_Face f2 = TopoDS::Face(s2);
        BRepAdaptor_Surface sf1(f1);
        BRepAdaptor_Surface sf2(f2);

        // First check that surface types are identical
        GeomAbs_SurfaceType t1 = sf1.GetType();
        GeomAbs_SurfaceType t2 = sf2.GetType();
        if (t1 != t2)
            return false;

        switch (t1) {
        case GeomAbs_Plane:
        {
            // Compare all vertices of the planar face
            TopTools_IndexedMapOfShape firstM, secondM;
            TopExp::MapShapes(s1, TopAbs_VERTEX, firstM);
            TopExp::MapShapes(s2, TopAbs_VERTEX, secondM);
            if (firstM.Extent() != secondM.Extent())
                return false;

            // TODO: Is it safe to assume that the vertex ordering must be identical?
            for (int i=1; i<=firstM.Extent(); i++) {
                gp_Pnt p1 = BRep_Tool::Pnt(TopoDS::Vertex(firstM(i)));
                gp_Pnt p2 = BRep_Tool::Pnt(TopoDS::Vertex(secondM(i)));
                if (!p1.IsEqual(p2, Precision::Confusion()))
                    return false;
            }

            return true;
        }
        default:
        {
            throw Base::Exception("Comparing faces other than planes is not implemented yet");
        }
        }
    }

    return false;
}

/// find the shape in the map
// Note: IsPartner is too general - e.g. it returns true for comparison of bottom and top face of a prism
// IsEqual is too specific - it returns false comparing equivalent faces
// Note: m->FindIndex(shape) will NOT work!
ShapeRef findRef(const std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> M, const TopoDS_Shape& shape)
{
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*>::const_iterator M_it = M.find(shape.ShapeType());
    if (M_it == M.end())
        // Note: This CAN happen e.g. for a PrismMaker where the sketchface generates a TopAbs_SOLID
        return ShapeRef();
    TopTools_IndexedMapOfShape* m = M_it->second;

    for (int i=1; i <= m->Extent(); ++i) {
        if ((*m)(i).IsSame(shape)) {
            ShapeRef result(shape.ShapeType(), i-1);
            return result;
        }
    }

    return ShapeRef();
}

// This is what is NOT being handled here, in order to keep the complexity down:
// BRepFilletAPI_MakeFillet::NewFaces()
//   Reason: We need to somehow define what "generated" these faces (plus their edges and vertices)
// BRepAlgoAPI_Fuse::SectionEdges() ::Modified2()
// BRepAlgoAPI_Cut::SectionEdges() ::Modified2()
//  Reason: The SectionEdges() were generated by a pair of faces, one from each solid in the boolean operation
//          We need to a) find those faces and b) enable mapping a set of faces to a single shape in curOldToCurNew
// BRepFeat_MakePrism::FirstShape() ::LastShape()
//   Reason: Unfortunately, these don't take an old shape as argument, in contrast to BRepPrimAPI_*
// BRepFeat_MakePrism::NewEdges() ::TgtEdges()
//   Reason: The newEdges() were generated by a pair of faces from prism and support shape (same as with booleans)
// AddShape and SubShape properties of Additive and Subtractive features are not included in the mapping at all
// When they are used in FeatureTransformed, the compounding of the transformed shapes is not handled, so only the
// path for the first transformed shape will be handled
std::map<ShapeRef, std::vector<ShapeRef> > mapCurOldToCurNew(BRepBuilderAPI_MakeShape* mkShape,
                                                             const TopoDS_Shape& oldShape)
{
    std::map<ShapeRef, std::vector<ShapeRef> > result;
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> newM = extractSubShapes(mkShape->Shape());
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> oldM = extractSubShapes(oldShape);

    for (int t = 0; t < numShapeTypes; ++t) {
        ShapeRef oldRef(shapeTypes[t]);

        for (int i=1; i<=oldM.at(shapeTypes[t])->Extent(); ++i) {
            oldRef.index = i-1; // adjust indices to start at zero
            TopoDS_Shape oldSh = oldRef.toShape(oldM);
            bool found = false;

            // Find all new shapes that are a modification of the old shape (e.g. a face was resized)
            for (TopTools_ListIteratorOfListOfShape it(mkShape->Modified(oldSh)); it.More(); it.Next()) {
                // Note: One oldSh can result in several modified shapes, e.g. if an entity was split into segments
                // We don't handle this case and simply take the first one
                ShapeRef newRef = findRef(newM, it.Value());
                if (!newRef.isEmpty()) {
                    result[oldRef].push_back(newRef);
                    found = true;
                    break;
                }
            }

            // Find all new shapes that were generated from an old shape (e.g. a face generated from an edge)
            for (TopTools_ListIteratorOfListOfShape it(mkShape->Generated(oldSh)); it.More(); it.Next()) {
                ShapeRef newRef = findRef(newM, it.Value());
                if (!newRef.isEmpty())
                    result[oldRef].push_back(newRef);
                    // Note: Don't mark as found because a generated shape is not the same element as oldSh
            }

            // Handle FirstShape() and LastShape() for prism and revolution classes
            BRepPrimAPI_MakePrism* mkPrism = dynamic_cast<BRepPrimAPI_MakePrism*>(mkShape);
            if (mkPrism != NULL) {
                ShapeRef newRef = findRef(newM, mkPrism->FirstShape(oldSh));
                if (!newRef.isEmpty()) {
                    result[oldRef].push_back(newRef);
                    found = true;
                }
                newRef = findRef(newM, mkPrism->LastShape(oldSh));
                if (!newRef.isEmpty())
                    result[oldRef].push_back(newRef);
                    // Note: Don't mark as found because LastShape is not the same element as oldSh
            }
            BRepPrimAPI_MakeRevol* mkRevol = dynamic_cast<BRepPrimAPI_MakeRevol*>(mkShape);
            if (mkRevol != NULL) {
                ShapeRef newRef = findRef(newM, mkRevol->FirstShape(oldSh));
                if (!newRef.isEmpty()) {
                    result[oldRef].push_back(newRef);
                    found = true;
                }
                newRef = findRef(newM, mkRevol->LastShape(oldSh));
                if (!newRef.isEmpty())
                    result[oldRef].push_back(newRef);
            }

            // This case is actually the most frequent one as all shapes that have not been
            // touched by the mkShape operation are handled here
            if (!found) {
                // Note: BRepFilletAPI_MakeFillet.IsDeleted() claims far too many shapes as deleted, so we
                // can't use IsDeleted() here to avoid unnecessary findRef() calls
                ShapeRef newRef = findRef(newM, oldSh);
                if (!newRef.isEmpty())
                    result[oldRef].push_back(newRef);
            }
        }
    }

    clearSubShapes(oldM);
    clearSubShapes(newM);
    return result;
}

void Feature::buildMaps(BRepBuilderAPI_MakeShape* builder, const std::vector<TopoDS_Shape>& oldShapes, const bool concatenate)
{
    // Initialize curOldToCurNew on first call
    if (curOldToCurNew.empty())
        for (unsigned idx = 0; idx < oldShapes.size(); ++idx)
            curOldToCurNew.push_back(RefMultiMap());

    if (!concatenate) {
        // curOldToCurNew now becomes prevNewToPrevOld by inverting the map
        prevNewToPrevOld.clear();
        for (unsigned idx = 0; idx < curOldToCurNew.size(); ++idx) {
            prevNewToPrevOld.push_back(RefMap());
            for (RefMultiMap::const_iterator m = curOldToCurNew[idx].begin(); m != curOldToCurNew[idx].end(); ++m)
                for (RefVec::const_iterator r = m->second.begin(); r != m->second.end(); ++r)
                    prevNewToPrevOld[idx][*r] = m->first;
        }

        // Clean and rebuild curOldToCurNew
        for (unsigned idx = 0; idx < oldShapes.size(); ++idx)
            curOldToCurNew[idx].clear();
    }

    // Build the maps
    std::vector<RefMultiMap> cotcn;
    for (unsigned idx = 0; idx < oldShapes.size(); ++idx)
        cotcn.push_back(mapCurOldToCurNew(builder, oldShapes[idx]));

    // Concatenate if required (and possible)
    if (concatenate) {
        std::vector<RefMultiMap> old_cotcn;
        old_cotcn.swap(curOldToCurNew);

        for (unsigned idx = 0; idx < oldShapes.size(); ++idx) {
            if (old_cotcn.size() <= idx) {
                // This happens for mixed operations, e.g. Pad:
                // Path one is from sketchshape to prism (BRepPrimAPI_MakePrism) to final shape (BRepAlgoAPI_Fuse)
                // Path two is from base shape to final shape (BRepAlgoAPI_Fuse)
                curOldToCurNew.push_back(cotcn[idx]);
                continue;
            }

            curOldToCurNew.push_back(RefMultiMap());

            for (RefMultiMap::const_iterator first_mm = old_cotcn[idx].begin(); first_mm != old_cotcn[idx].end(); ++first_mm) {
                for (RefVec::const_iterator first_it = first_mm->second.begin(); first_it != first_mm->second.end(); ++first_it) {
                    RefMultiMap::const_iterator second_mm = cotcn[idx].find(*first_it);
                    if (second_mm != cotcn[idx].end()) {
                        for (RefVec::const_iterator r = second_mm->second.begin(); r != second_mm->second.end(); ++r)
                            curOldToCurNew[idx][first_mm->first].push_back(*r);
                    }
                }
            }
        }
    } else {
        curOldToCurNew.swap(cotcn);
    }
}

void Feature::buildMaps(BRepBuilderAPI_MakeShape* mkShape, const TopoDS_Shape& oldShape)
{
    std::vector<TopoDS_Shape> oldShapes;
    oldShapes.push_back(oldShape);
    buildMaps(mkShape, oldShapes);
}

void Feature::buildMaps(BRepBuilderAPI_MakeShape* mkShape,
                        const TopoDS_Shape& oldShape1, const TopoDS_Shape& oldShape2,
                        const bool concatenate)
{
    std::vector<TopoDS_Shape> oldShapes;
    oldShapes.push_back(oldShape1);
    oldShapes.push_back(oldShape2);
    buildMaps(mkShape, oldShapes, concatenate);
}

void Feature::remapProperties(const std::vector<const Part::Feature*>& oldFeatures)
{
    // Build map prevToCurrent from previous to current shape for fast lookups
    if (prevNewToPrevOld.size() != oldFeatures.size())
        throw Base::Exception("Feature::remapProperties(): Internal error: Path size mismatch\n");

    //Base::Console().Error("Map for %s\n", this->getNameInDocument());
    prevToCurrent.clear();
    for (unsigned idx = 0; idx < prevNewToPrevOld.size(); ++idx) {
        std::map<ShapeRef, int> multi_idx; // For handling multiple new shapes in curOldToCurNew

        for (RefMap::const_iterator m = prevNewToPrevOld[idx].begin(); m != prevNewToPrevOld[idx].end(); ++m) {
            ShapeRef curOld = oldFeatures[idx]->convertPrevToCur(m->second);
            RefMultiMap::const_iterator curNew_it = curOldToCurNew[idx].find(curOld);
            if (curNew_it != curOldToCurNew[idx].end()) {
                ShapeRef curNew;
                if (curNew_it->second.size() == 1) {
                    curNew = curNew_it->second.front();
                } else {
                    std::map<ShapeRef, int>::const_iterator idx_it = multi_idx.find(curOld);
                    int refidx = (idx_it == multi_idx.end() ? 0 : idx_it->second);
                    if (refidx >= curNew_it->second.size())
                        continue;
                    curNew = curNew_it->second[refidx];
                    multi_idx[curOld] = refidx+1; // Remember that we have already used this shape
                }
                if (curNew.isEmpty())
                    int x = 1;
                prevToCurrent[m->first] = curNew;
                //Base::Console().Error("%s ==> %s\n", m->first.toSubName().c_str(), curNew.toSubName().c_str());
            }
        }
    }

    // Update all dependent properties
    updateProperties();
}

void Feature::remapProperties(const Part::Feature* oldFeature)
{
    std::vector<const Part::Feature*> oldFeatures;
    oldFeatures.push_back(oldFeature);
    remapProperties(oldFeatures);
}

void Feature::remapProperties(const Part::Feature* oldFeature1, const Part::Feature* oldFeature2)
{
    std::vector<const Part::Feature*> oldFeatures;
    oldFeatures.push_back(oldFeature1);
    oldFeatures.push_back(oldFeature2);
    remapProperties(oldFeatures);
}

// What we don't handle here:
// Sketch elements that have changed their position on the sketch plane without the topology being changed
//    Reason: The easiest way to solve this is to assign a unique ID to every sketch element so that we can
//            always find it again no matter where the user moves it (see my TNaming branch)
// A curve between two sketch points that was deleted and replaced by (one or more) other sketch elements
//    Reason: To achieve this the sketcher itself must provide the information to the mapping algorithm
void Feature::remapProperties(const TopoDS_Shape& prevShape, const TopoDS_Shape& curShape)
{
    prevToCurrent.clear();

    // Extract all subshapes from previous and current shape
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> prevM = extractSubShapes(prevShape);
    std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*> curM = extractSubShapes(curShape);

    // Look at all objects in the old shape and try to find the related object in the new shape
    // Note: IsSame() is unlikely to work here since the Sketch always rebuilds its shape from scratch
    //Base::Console().Error("Map for %s\n", this->getNameInDocument());
    for (int t = 0; t < numShapeTypes; ++t) {
        TopAbs_ShapeEnum type = shapeTypes[t];
        ShapeRef prevRef(type);

        for (int i=1; i<=prevM[type]->Extent(); ++i) {
            prevRef.index = i-1; // adjust indices to start at zero
            TopoDS_Shape prevShape = prevRef.toShape(prevM);

            for (int j=1; j<=curM[type]->Extent(); ++j) { // find object in curM corresponding to prevM
                ShapeRef curRef(type, j-1);
                if (compareShapes(prevShape, curRef.toShape(curM), type)) {
                    prevToCurrent[prevRef] = curRef;
                    //Base::Console().Error("%s ==> %s\n", prevRef.toSubName().c_str(), curRef.toSubName().c_str());
                    break;
                }
            }
        }
    }

    clearSubShapes(curM);
    clearSubShapes(prevM);

    // Update all dependent properties
    updateProperties();
}

const ShapeRef Feature::convertPrevToCur(const ShapeRef& ref) const
{
    RefMap::const_iterator r = prevToCurrent.find(ref);
    if ((r != prevToCurrent.end() && (ref.type == r->second.type))) {
        if (r->second.isEmpty())
            int x = int(ref.type);
        return r->second;
    } else
        return ref;
}

void Feature::updateProperty(App::PropertyLinkSub* prop)
{
    if (prop->getValue() != this)
        return;

    std::vector<std::string> subs = prop->getSubValues();
    std::vector<std::string> newsubs;

    for (std::vector<std::string>::const_iterator s = subs.begin(); s != subs.end(); ++s) {
        newsubs.push_back(convertPrevToCur(*s));
        //Base::Console().Error("Property subname changed from %s to %s\n", s->c_str(), newsubs.back().c_str());
    }

    prop->setValue(this, newsubs);
}

void Feature::updateProperty(App::PropertyLinkSubList* prop)
{
    std::vector<App::DocumentObject*> links = prop->getValues();
    std::vector<std::string> subs = prop->getSubValues();
    std::vector<std::string> newsubs;

    for (unsigned i = 0; i < links.size(); ++i) {
        if (links[i] == this) {
            newsubs.push_back(convertPrevToCur(subs[i]));
            //Base::Console().Error("Property subname changed from %s to %s\n", subs[i].c_str(), newsubs.back().c_str());
        } else {
            newsubs.push_back(subs[i]);
        }
    }

    prop->setValues(links, newsubs);
}

void Feature::updateProperties()
{
    std::vector<App::DocumentObject*> objs = this->getInList();
    std::set<App::DocumentObject*> handled_objs;
    for (std::vector<App::DocumentObject*>::iterator o = objs.begin(); o != objs.end(); ++o) {
        if (handled_objs.find(*o) != handled_objs.end())
            continue;
        else
            handled_objs.insert(*o);

        std::vector<App::Property*> props;
        (*o)->getPropertyList(props);
        for (std::vector<App::Property*>::iterator p = props.begin(); p != props.end(); ++p) {
            if ((*p)->isDerivedFrom(App::PropertyLinkSub::getClassTypeId()))
                updateProperty(static_cast<App::PropertyLinkSub*>(*p));
            else if ((*p)->isDerivedFrom(App::PropertyLinkSubList::getClassTypeId()))
                updateProperty(static_cast<App::PropertyLinkSubList*>(*p));
        }
    }
}
