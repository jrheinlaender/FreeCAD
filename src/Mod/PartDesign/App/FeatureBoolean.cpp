/******************************************************************************
 *   Copyright (c)2013 Jan Rheinlaender <jrheinlaender@users.sourceforge.net> *
 *                                                                            *
 *   This file is part of the FreeCAD CAx development system.                 *
 *                                                                            *
 *   This library is free software; you can redistribute it and/or            *
 *   modify it under the terms of the GNU Library General Public              *
 *   License as published by the Free Software Foundation; either             *
 *   version 2 of the License, or (at your option) any later version.         *
 *                                                                            *
 *   This library  is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU Library General Public License for more details.                     *
 *                                                                            *
 *   You should have received a copy of the GNU Library General Public        *
 *   License along with this library; see the file COPYING.LIB. If not,       *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,            *
 *   Suite 330, Boston, MA  02111-1307, USA                                   *
 *                                                                            *
 ******************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
# include <BRepAlgoAPI_Fuse.hxx>
# include <BRepAlgoAPI_Cut.hxx>
# include <BRepAlgoAPI_Common.hxx>
# include <BRepAlgoAPI_Section.hxx>
# include <Standard_Failure.hxx>
# include <gp_Trsf.hxx>
# include <gp_Pnt.hxx>
# include <gp_Ax1.hxx>
# include <gp_Vec.hxx>
#endif

#include "Body.h"
#include "FeatureBoolean.h"

#include <Base/Console.h>
#include <Base/Exception.h>

using namespace PartDesign;

namespace PartDesign {

PROPERTY_SOURCE(PartDesign::Boolean, PartDesign::Feature)

const char* Boolean::TypeEnums[]= {"Fuse","Cut","Common","Section",NULL};

Boolean::Boolean()
{
    ADD_PROPERTY(Type,((long)0));
    Type.setEnums(TypeEnums);
    ADD_PROPERTY(Bodies,(0));
    Bodies.setSize(0);
}

short Boolean::mustExecute() const
{
    if (Bodies.isTouched())
        return 1;
    return PartDesign::Feature::mustExecute();
}

App::DocumentObjectExecReturn *Boolean::execute(void)
{
    // Get the base shape to operate on
    Part::TopoShape theBoolean = getBaseTopoShape();

    std::vector<App::DocumentObject*> bodies = Bodies.getValues();
    if (bodies.empty())
        return App::DocumentObject::StdReturn;

    // Remove the transformation of the base shape
    theBoolean.setTransform(Base::Matrix4D());

    try {
        // Position this feature by the first body
        const Part::Feature* baseFeature = getBaseObject();
        this->Placement.setValue(baseFeature->Placement.getValue());
        TopLoc_Location invObjLoc = this->getLocation().Inverted();

        // Get the operation type
        std::string type = Type.getValueAsString();

        for (std::vector<App::DocumentObject*>::const_iterator b = bodies.begin(); b != bodies.end(); b++)
        {
            // Extract the body toposhape
            PartDesign::Body* body = static_cast<PartDesign::Body*>(*b);
            Part::Feature* tipSolid = static_cast<Part::Feature*>(body->getPrevSolidFeature());
            if (tipSolid == NULL)
                continue;
            Part::TopoShape other = tipSolid->Shape.getShape();

            // Move the shape to the location of the base shape
            Base::Placement pl = body->Placement.getValue();
            // TODO: Why is Feature::getLocation() protected?
            Base::Rotation rot(pl.getRotation());
            Base::Vector3d axis;
            double angle;
            rot.getValue(axis, angle);
            gp_Trsf trf;
            trf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(axis.x, axis.y, axis.z)), angle);
            trf.SetTranslationPart(gp_Vec(pl.getPosition().x,pl.getPosition().y,pl.getPosition().z));
            TopLoc_Location bLoc(trf);
            other.move(invObjLoc.Multiplied(bLoc));

            if (type == "Fuse")
                theBoolean.makeFuse(other);
            else if (type == "Cut")
                theBoolean.makeCut(other);
            else if (type == "Common")
                theBoolean.makeCommon(other);
            else if (type == "Section")
                theBoolean.makeSection(other);
        }

        this->Shape.setValue(theBoolean);
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

}
