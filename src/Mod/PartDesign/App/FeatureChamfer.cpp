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
# include <BRepFilletAPI_MakeChamfer.hxx>
# include <TopExp.hxx>
# include <TopExp_Explorer.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Edge.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#endif

#include <Base/Exception.h>
#include <Mod/Part/App/TopoShape.h>

#include "FeatureChamfer.h"


using namespace PartDesign;


PROPERTY_SOURCE(PartDesign::Chamfer, PartDesign::DressUp)

const App::PropertyFloatConstraint::Constraints floatSize = {0.0,FLT_MAX,0.1};

Chamfer::Chamfer()
{
    ADD_PROPERTY(Size,(1.0));
    Size.setConstraints(&floatSize);
}

short Chamfer::mustExecute() const
{
    if (Placement.isTouched() || Size.isTouched())
        return 1;
    return DressUp::mustExecute();
}

App::DocumentObjectExecReturn *Chamfer::execute(void)
{
    // NOTE: Normally the Base property and the BaseFeature property should point to the same object.
    // The only difference is that the Base property also stores the edges that are to be chamfered
    App::DocumentObject* link = BaseFeature.getValue();
    if (!link)
        link = Base.getValue(); // For legacy features
    if (!link)
        return new App::DocumentObjectExecReturn("No object linked");
    if (!link->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId()))
        return new App::DocumentObjectExecReturn("Linked object is not a Part object");
    Part::TopoShape theChamfer = static_cast<Part::Feature*>(link)->Shape.getShape();
    if (theChamfer._Shape.IsNull())
        return new App::DocumentObjectExecReturn("Cannot chamfer invalid shape");

    const std::vector<std::string>& SubVals = Base.getSubValuesStartsWith("Edge");
    if (SubVals.size() == 0)
        return new App::DocumentObjectExecReturn("No edges specified");

    double size = Size.getValue();

    this->positionByBaseFeature();
    // untransform the basefeature shape
    theChamfer.setTransform(Base::Matrix4D());

    try {
        theChamfer.makeChamfer(SubVals, size);
        if (checkRefineActive())
            theChamfer.refine();
        this->Shape.setValue(theChamfer);
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
