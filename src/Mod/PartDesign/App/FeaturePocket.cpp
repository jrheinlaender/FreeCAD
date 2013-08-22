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
# include <Bnd_Box.hxx>
# include <gp_Dir.hxx>
# include <gp_Pln.hxx>
# include <BRep_Builder.hxx>
# include <BRepAdaptor_Surface.hxx>
# include <BRepBndLib.hxx>
# include <BRepFeat_MakePrism.hxx>
# include <BRepBuilderAPI_MakeFace.hxx>
# include <Geom_Plane.hxx>
# include <Handle_Geom_Surface.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Wire.hxx>
# include <TopoDS_Solid.hxx>
# include <TopExp_Explorer.hxx>
# include <BRepAlgoAPI_Cut.hxx>
# include <BRepPrimAPI_MakeHalfSpace.hxx>
# include <BRepAlgoAPI_Common.hxx>
#endif

#include <Base/Exception.h>
#include <Base/Placement.h>
#include <App/Document.h>

#include "FeaturePocket.h"


using namespace PartDesign;

const char* Pocket::TypeEnums[]= {"Length","ThroughAll","UpToFirst","UpToFace",NULL};

PROPERTY_SOURCE(PartDesign::Pocket, PartDesign::Subtractive)

Pocket::Pocket()
{
    ADD_PROPERTY(Type,((long)0));
    Type.setEnums(TypeEnums);
    ADD_PROPERTY(Length,(100.0));
    ADD_PROPERTY(Offset,(0.0));
}

short Pocket::mustExecute() const
{
    if (Placement.isTouched() ||
        Type.isTouched() ||
        Length.isTouched())
        return 1;
    return Subtractive::mustExecute();
}

App::DocumentObjectExecReturn *Pocket::execute(void)
{
    std::string method(Type.getValueAsString());

    // Handle legacy features, these typically have Type set to 3 (previously NULL, now UpToFace),
    // empty FaceName (because it didn't exist) and a value for Length
    if (method == "UpToFace" &&
        (UpToFace.getValue() == NULL && Length.getValue() > Precision::Confusion()))
        Type.setValue("Length");

    // Validate parameters
    double L = Length.getValue();
    if ((method == "Length") && (L < Precision::Confusion()))
        return new App::DocumentObjectExecReturn("Pocket: Length of pocket too small");
    if (method == "ThroughAll")
        L = Precision::Infinite();

    Part::Part2DObject* sketch;
    try {
        sketch = getVerifiedSketch();
    } catch (const Base::Exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }
    const Part::TopoShape& theSketch = sketch->Shape.getShape();

    // Get the BaseFeature TopoShape (if any)
    Part::TopoShape theBase;
    try {
        theBase = getBaseTopoShape();
    } catch (const Base::Exception&) {
        try {
            // fall back to support (for legacy features)
            theBase = getSupportTopoShape();
        } catch (const Base::Exception&) {
            return new App::DocumentObjectExecReturn("No sketch support and no base shape: Please tell me where to remove the material of the pocket!");
        }
    }

    // get the Sketch plane
    Base::Placement SketchPos = sketch->Placement.getValue();
    Base::Rotation SketchOrientation = SketchPos.getRotation();
    Base::Vector3d SketchVector(0,0,1);
    SketchOrientation.multVec(SketchVector,SketchVector);

    // turn around for pockets
    SketchVector *= -1;

    try {
        this->positionBySketch();
        TopLoc_Location invObjLoc = this->getLocation().Inverted();

        gp_Dir dir(SketchVector.x,SketchVector.y,SketchVector.z);
        dir.Transform(invObjLoc.Transformation());

        Part::TopoShape thePocket = theSketch;
        thePocket.makeFace();

        thePocket.move(invObjLoc);
        theBase.move(invObjLoc);

        if (method == "UpToFirst" || method == "UpToFace") {
            if (base.IsNull())
                return new App::DocumentObjectExecReturn("Pocket: Extruding up to a face is only possible if the sketch is located on a face");

        if (method == "UpToFirst" || method == "UpToFace") {
            TopoDS_Face supportface = getSupportFace();
            supportface.Move(invObjLoc);

            if (Reversed.getValue())
                dir.Reverse();

            // Find a valid face or datum plane to extrude up to
            TopoDS_Face upToFace;
            if (method == "UpToFace") {
                getUpToFaceFromLinkSub(upToFace, UpToFace);
                upToFace.Move(invObjLoc);
            }
            getUpToFace(upToFace, theBase._Shape, supportface, thePocket._Shape, method, dir, Offset.getValue());

            thePocket.makePrism(theBase, supportface, dir, upToFace, true);

            // And the really expensive way to get the SubShape...
            // FIXME: Whether we use Cut or Section, the result is always an empty shape (no solids)
            //        Probably there are too many identical shapes for the boolean operation to work properly
            Part::TopoShape theSub = theBase;
            theSub.makeCut(thePocket);
            if (checkRefineActive())
                theSub.refine();

            this->SubShape.setValue(theSub);
        } else {
            thePocket.makePrism(dir, L, 0.0, Midplane.getValue(), Reversed.getValue());

            // set the subtractive shape property for later usage in e.g. pattern
            if (checkRefineActive())
                thePocket.refine();
            this->SubShape.setValue(thePocket);

            // Cut out of base
            thePocket.makeCut(theBase, false);

            remapSupportShape(thePocket._Shape);
        }

        if (checkRefineActive())
            thePocket.refine();
        this->Shape.setValue(thePocket);

        return App::DocumentObject::StdReturn;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        return new App::DocumentObjectExecReturn(e->GetMessageString());
    }
    catch (Base::Exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }
}

