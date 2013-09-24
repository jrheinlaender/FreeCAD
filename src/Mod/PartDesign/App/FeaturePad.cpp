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
# include <BRep_Builder.hxx>
# include <BRep_Tool.hxx>
# include <BRepBndLib.hxx>
# include <BRepFeat_MakePrism.hxx>
# include <BRepBuilderAPI_MakeFace.hxx>
# include <Handle_Geom_Surface.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Solid.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Wire.hxx>
# include <TopExp_Explorer.hxx>
# include <BRepAlgoAPI_Fuse.hxx>
# include <Precision.hxx>
# include <BRepPrimAPI_MakeHalfSpace.hxx>
# include <BRepAlgoAPI_Common.hxx>
# include <BRepAdaptor_Surface.hxx>
# include <gp_Pln.hxx>
# include <GeomAPI_ProjectPointOnSurf.hxx>
#endif

#include <Base/Exception.h>
#include <Base/Placement.h>
#include <App/Document.h>

//#include "Body.h"
#include "FeaturePad.h"


using namespace PartDesign;

const char* Pad::TypeEnums[]= {"Length","UpToLast","UpToFirst","UpToFace","TwoLengths",NULL};

PROPERTY_SOURCE(PartDesign::Pad, PartDesign::Additive)

Pad::Pad()
{
    ADD_PROPERTY(Type,((long)0));
    Type.setEnums(TypeEnums);
    ADD_PROPERTY(Length,(100.0));
    ADD_PROPERTY(Length2,(100.0));
    ADD_PROPERTY(Offset,(0.0));
}

short Pad::mustExecute() const
{
    if (Placement.isTouched() ||
        Type.isTouched() ||
        Length.isTouched() ||
        Length2.isTouched())
        return 1;
    return Additive::mustExecute();
}

App::DocumentObjectExecReturn *Pad::execute(void)
{
    // Validate parameters
    std::string method(Type.getValueAsString());
    double L = Length.getValue();
    double L2 = Length2.getValue();
    if (method == "Length") {
        if (L < Precision::Confusion())
            return new App::DocumentObjectExecReturn("Length of pad too small");
        L2 = 0.0; // Just to avoid confusing the algorithm
    }
    if ((method == "TwoLengths") && (L2 < Precision::Confusion()))
        return new App::DocumentObjectExecReturn("Second length of pad too small");

    // Get the sketch TopoShape
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
            // ignore, because base isn't mandatory
        }
    }

/*
    // Find Body feature which owns this Pad and get the shape of the feature preceding this one for fusing
    // This method was rejected in favour of the BaseFeature property because that makes the feature atomic (independent of the
    // Body object). See
    // https://sourceforge.net/apps/phpbb/free-cad/viewtopic.php?f=19&t=3831
    // https://sourceforge.net/apps/phpbb/free-cad/viewtopic.php?f=19&t=3855
    PartDesign::Body* body = getBody();
    if (body == NULL) {
        return new App::DocumentObjectExecReturn(
                    "In order to use PartDesign you need an active Body object in the document. "
                    "Please make one active or create one. If you have a legacy document "
                    "with PartDesign objects without Body, use the transfer function in "
                    "PartDesign to put them into a Body."
                    );
    }
    const Part::TopoShape& prevShape = body->getPreviousSolid(this);
    TopoDS_Shape support;
    if (prevShape.isNull())
        // ignore, because support isn't mandatory
        support = TopoDS_Shape();
    else
        support = prevShape._Shape;
*/

    // get the Sketch plane
    Base::Placement SketchPos = sketch->Placement.getValue();
    Base::Rotation SketchOrientation = SketchPos.getRotation();
    Base::Vector3d SketchVector(0,0,1);
    SketchOrientation.multVec(SketchVector,SketchVector);

    try {        
        this->positionBySketch();
        TopLoc_Location invObjLoc = this->getLocation().Inverted();

        gp_Dir dir(SketchVector.x,SketchVector.y,SketchVector.z);
        dir.Transform(invObjLoc.Transformation());

        Part::TopoShape thePad = theSketch;
        thePad.makeFace();

        thePad.move(invObjLoc);
        theBase.move(invObjLoc);

        if (method == "UpToFirst" || method == "UpToLast" || method == "UpToFace") {
              // Note: This will return an unlimited planar face if support is a datum plane
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
            getUpToFace(upToFace, theBase._Shape, supportface, thePad._Shape, method, dir, Offset.getValue());

            thePad.makePrism(theBase, supportface, dir, upToFace);
        } else {
            thePad.makePrism(dir, L, L2, Midplane.getValue(), Reversed.getValue());
        }

        // set the additive shape property for later usage in e.g. pattern
        this->AddShape.setValue(thePad);

        // Fuse with base (the algorithm will simply ignore an empty theBase and change nothing)
        thePad.makeFuse(theBase, false);

        this->Shape.setValue(thePad);

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

