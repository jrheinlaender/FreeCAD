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
# include <BRepBndLib.hxx>
# include <BRepPrimAPI_MakeRevol.hxx>
# include <BRepBuilderAPI_MakeFace.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Wire.hxx>
# include <TopExp_Explorer.hxx>
# include <BRepAlgoAPI_Fuse.hxx>
# include <Precision.hxx>
# include <gp_Lin.hxx>
#endif

#include <Base/Axis.h>
#include <Base/Exception.h>
#include <Base/Placement.h>
#include <Base/Tools.h>

#include "FeatureRevolution.h"


using namespace PartDesign;

namespace PartDesign {


PROPERTY_SOURCE(PartDesign::Revolution, PartDesign::Additive)

Revolution::Revolution()
{
    ADD_PROPERTY_TYPE(Base,(Base::Vector3d(0.0,0.0,0.0)),"Revolution", App::Prop_ReadOnly, "Base");
    ADD_PROPERTY_TYPE(Axis,(Base::Vector3d(0.0,1.0,0.0)),"Revolution", App::Prop_ReadOnly, "Axis");
    ADD_PROPERTY_TYPE(Angle,(360.0),"Revolution", App::Prop_None, "Angle");
    ADD_PROPERTY_TYPE(ReferenceAxis,(0),"Revolution",(App::Prop_None),"Reference axis of revolution");
}

short Revolution::mustExecute() const
{
    if (Placement.isTouched() ||
        ReferenceAxis.isTouched() ||
        Axis.isTouched() ||
        Base.isTouched() ||
        Angle.isTouched())
        return 1;
    return Additive::mustExecute();
}

App::DocumentObjectExecReturn *Revolution::execute(void)
{
    // Validate parameters
    double angle = Angle.getValue();
    if (angle < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Angle of revolution too small");
    if (angle > 360.0)
        return new App::DocumentObjectExecReturn("Angle of revolution too large");

    angle = Base::toRadians<double>(angle);

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

    // update Axis from ReferenceAxis
    updateAxis();

    // get revolve axis
    Base::Vector3d b = Base.getValue();
    gp_Pnt pnt(b.x,b.y,b.z);
    Base::Vector3d v = Axis.getValue();
    gp_Dir dir(v.x,v.y,v.z);

    try {
        Part::TopoShape theRevolution = theSketch;
        theRevolution.makeFace();

        this->positionBySketch();
        TopLoc_Location invObjLoc = this->getLocation().Inverted();
        pnt.Transform(invObjLoc.Transformation());
        dir.Transform(invObjLoc.Transformation());
        theRevolution.move(invObjLoc);
        theBase.move(invObjLoc);

        theRevolution.makeRevolution(gp_Ax1(pnt, dir), angle, Midplane.getValue(), Reversed.getValue());

        // set the additive shape property for later usage in e.g. pattern
        if (checkRefineActive())
            theRevolution.refine();
        this->AddShape.setValue(theRevolution);

        // Fuse with base (the algorithm will simply ignore an empty theBase and change nothing)
        theRevolution.makeFuse(theBase, false);
        if (checkRefineActive())
            theRevolution.refine();

        this->Shape.setValue(theRevolution);

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

bool Revolution::suggestReversed(void)
{
    updateAxis();
    return SketchBased::getReversedAngle(Base.getValue(), Axis.getValue()) < 0.0;
}

void Revolution::updateAxis(void)
{
    App::DocumentObject *pcReferenceAxis = ReferenceAxis.getValue();
    const std::vector<std::string> &subReferenceAxis = ReferenceAxis.getSubValues();
    Base::Vector3d base;
    Base::Vector3d dir;
    getAxis(pcReferenceAxis, subReferenceAxis, base, dir);

    Base.setValue(base.x,base.y,base.z);
    Axis.setValue(dir.x,dir.y,dir.z);
}

}
