/***************************************************************************
 *   Copyright (c) 2011 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
# include <Standard_Failure.hxx>
#endif

#include <Base/Placement.h>
#include <Base/Exception.h>

#include "FeatureFace.h"


using namespace PartDesign;


PROPERTY_SOURCE(PartDesign::Face, Part::Part2DObject)

Face::Face()
{
    ADD_PROPERTY(Sources,(0));
    Sources.setSize(0);
}

short Face::mustExecute() const
{
    if (Sources.isTouched())
        return 1;
    return Part::Part2DObject::mustExecute();
}

App::DocumentObjectExecReturn *Face::execute(void)
{
    std::vector<App::DocumentObject*> links = Sources.getValues();
    if (links.empty())
        return new App::DocumentObjectExecReturn("No shapes linked");

    std::vector<App::DocumentObject*>::iterator it = links.begin();
    if (!((*it)->isDerivedFrom(Part::Part2DObject::getClassTypeId())))
        return new App::DocumentObjectExecReturn("Linked object is not a Sketch or Part2DObject");
    Part::TopoShape theFace = static_cast<Part::Part2DObject*>(*it)->Shape.getShape();
    it++;

    try {
        for (; it != links.end(); ++it) {
            if (!(*it && (*it)->isDerivedFrom(Part::Part2DObject::getClassTypeId())))
                return new App::DocumentObjectExecReturn("Linked object is not a Sketch or Part2DObject");

            theFace.makeCompound(static_cast<Part::Part2DObject*>(*it)->Shape.getShape());
        }

        theFace.makeFace();

        this->Shape.setValue(theFace);
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
