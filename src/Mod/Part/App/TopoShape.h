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


#ifndef PART_TOPOSHAPE_H
#define PART_TOPOSHAPE_H

#include <iostream>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_ListOfShape.hxx>
#include <App/ComplexGeoData.h>

class gp_Ax1;
class gp_Ax2;
class gp_Vec;
class gp_Dir;
class gp_Pln;
class TopoDS_Face;

namespace Part
{
class Geometry; // Note: #include "Geometry.h" messes up ImportIges.cpp!
class Part2DObject;

class PartExport ShapeSegment : public Data::Segment
{
    TYPESYSTEM_HEADER();

public:
    ShapeSegment(const TopoDS_Shape &ShapeIn):Shape(ShapeIn){}
    ShapeSegment(){}
    virtual std::string getName() const;

    TopoDS_Shape Shape;
};



/** The representation for a CAD Shape
 */
class PartExport TopoShape : public Data::ComplexGeoData
{
    TYPESYSTEM_HEADER();

public:
    TopoShape();
    TopoShape(const TopoDS_Shape&);
    TopoShape(const TopoShape&);
    ~TopoShape();

    void operator = (const TopoShape&);

    /** @name Placement control */
    //@{
    /// set the transformation of the CasCade Shape
    void setTransform(const Base::Matrix4D& rclTrf);
    /// get the transformation of the CasCade Shape
    Base::Matrix4D getTransform(void) const;
    /// Bound box from the CasCade shape
    Base::BoundBox3d getBoundBox(void)const;
    static void convertTogpTrsf(const Base::Matrix4D& mtrx, gp_Trsf& trsf);
    static void convertToMatrix(const gp_Trsf& trsf, Base::Matrix4D& mtrx);
    //@}

    /** @name Subelement management */
    //@{
    /** Sub type list
     *  List of different subelement types
     *  it is NOT a list of the subelements itself
     */
    virtual std::vector<const char*> getElementTypes(void) const;
    virtual unsigned long countSubElements(const char* Type) const;
    /// get the subelement by type and number
    virtual Data::Segment* getSubElement(const char* Type, unsigned long) const;
    /** Get lines from segment */
    virtual void getLinesFromSubelement(
        const Data::Segment*,
        std::vector<Base::Vector3d> &Points,
        std::vector<Line> &lines) const;
    /** Get faces from segment */
    virtual void getFacesFromSubelement(
        const Data::Segment*,
        std::vector<Base::Vector3d> &Points,
        std::vector<Base::Vector3d> &PointNormals,
        std::vector<Facet> &faces) const;
    //@}
    /// get the Topo"sub"Shape with the given name
    TopoDS_Shape getSubShape(const char* Type) const;
    unsigned long countSubShapes(const char* Type) const;
    /// get the Topo"sub"Shape with the given name
    PyObject * getPySubShape(const char* Type) const;

    /** @name Save/restore */
    //@{
    void Save (Base::Writer &writer) const;
    void Restore(Base::XMLReader &reader);

    void SaveDocFile (Base::Writer &writer) const;
    void RestoreDocFile(Base::Reader &reader);
    unsigned int getMemSize (void) const;
    //@}

    /** @name Input/Output */
    //@{
    void read(const char *FileName);
    void write(const char *FileName) const;
    void importIges(const char *FileName);
    void importStep(const char *FileName);
    void importBrep(const char *FileName);
    void importBrep(std::istream&);
    void exportIges(const char *FileName) const;
    void exportStep(const char *FileName) const;
    void exportBrep(const char *FileName) const;
    void exportBrep(std::ostream&);
    void exportStl (const char *FileName) const;
    void exportFaceSet(double, double, std::ostream&) const;
    void exportLineSet(std::ostream&) const;
    //@}

    /** @name Query*/
    //@{
    bool isNull() const;
    bool isValid() const;
    bool analyze(std::ostream&) const;
    bool isClosed() const;
    /**
      * Check for intersection between the two shapes. Only solids are guaranteed to work properly
      * There are two modes:
      * 1. Bounding box check only - quick but inaccurate
      * 2. Bounding box check plus (if necessary) boolean operation - costly but accurate
      * Return true if the shapes intersect, false if they don't
      * The flag touch_is_intersection decides whether shapes touching at distance zero are regarded
      * as intersecting or not
      * 1. If set to true, a true check result means that a boolean fuse operation between the two shapes
      *    will return a single solid
      * 2. If set to false, a true check result means that a boolean common operation will return a
      *    valid solid
      * If there is any error in the boolean operations, the check always returns false
      */
    bool intersects(const TopoShape& other, const bool quick, const bool touch_is_intersection) const;
    //@}

    /** @name Boolean operation*/
    //@{
    TopoDS_Shape cut(TopoDS_Shape) const;
    TopoDS_Shape common(TopoDS_Shape) const;
    TopoDS_Shape fuse(TopoDS_Shape) const;
    TopoDS_Shape oldFuse(TopoDS_Shape) const;
    TopoDS_Shape section(TopoDS_Shape) const;
    std::list<TopoDS_Wire> slice(const Base::Vector3d&, double) const;
    TopoDS_Compound slices(const Base::Vector3d&, const std::vector<double>&) const;
    //@}

    /** Sweeping */
    //@{
    TopoDS_Shape makePipe(const TopoDS_Shape& profile) const;
    TopoDS_Shape makePipeShell(const TopTools_ListOfShape& profiles, const Standard_Boolean make_solid,
        const Standard_Boolean isFrenet = Standard_False, int transition=0) const;
    TopoDS_Shape makePrism(const gp_Vec&) const;
    TopoDS_Shape revolve(const gp_Ax1&, double d) const;
    TopoDS_Shape makeSweep(const TopoDS_Shape& profile, double, int) const;
    TopoDS_Shape makeTube(double radius, double tol, int cont, int maxdeg, int maxsegm) const;
    TopoDS_Shape makeHelix(Standard_Real pitch, Standard_Real height,
        Standard_Real radius, Standard_Real angle=0, Standard_Boolean left=Standard_False) const;
    TopoDS_Shape makeThread(Standard_Real pitch, Standard_Real depth,
        Standard_Real height, Standard_Real radius) const;
    TopoDS_Shape makeLoft(const TopTools_ListOfShape& profiles, Standard_Boolean isSolid,
        Standard_Boolean isRuled) const;
    TopoDS_Shape makeOffsetShape(double offset, double tol,
        bool intersection = false, bool selfInter = false,
        short offsetMode = 0, short join = 0, bool fill = false) const;
    TopoDS_Shape makeThickSolid(const TopTools_ListOfShape& remFace,
        double offset, double tol,
        bool intersection = false, bool selfInter = false,
        short offsetMode = 0, short join = 0) const;
    //@}

    /** @name Manipulation*/
    //@{
    void transformGeometry(const Base::Matrix4D &rclMat);
    TopoDS_Shape transformGShape(const Base::Matrix4D&) const;
    void transformShape(const Base::Matrix4D&, bool copy);
    TopoDS_Shape mirror(const gp_Ax2&) const;
    TopoDS_Shape toNurbs() const;
    TopoDS_Shape replaceShape(const std::vector< std::pair<TopoDS_Shape,TopoDS_Shape> >& s) const;
    TopoDS_Shape removeShape(const std::vector<TopoDS_Shape>& s) const;
    void sewShape();
    bool fix(double, double, double);
    bool removeInternalWires(double);
    TopoDS_Shape removeSplitter() const;
    //@}

    /** @name Getting basic geometric entities */
    //@{
    void getFaces(std::vector<Base::Vector3d> &Points,std::vector<Facet> &faces,
        float Accuracy, uint16_t flags=0) const;
    void setFaces(const std::vector<Base::Vector3d> &Points,
                  const std::vector<Facet> &faces, float Accuracy=1.0e-06);
    //@}

    /** @name Geometrical operations on TopoShape */
    /**
      * To make use of topological naming these methods must be used to manipulate the geometry
      * contained inside a TopoShape. Using the set of methods above which take a TopoDS_Shape
      * as an argument and return a TopoDS_Shape will NOT work with topological naming
      */
    //@{
    /**
      * Renew all relevant information about the Sketch in the TopoShape
      * This method will set the root of all history in the TopoShape
      * Throws an exception if the TopoShape contains anything except one or more wires
      */
    // Note: Since we don't want to move all the sketch creation code from Sketcher:: to here we
    // need to pass in both the geometry and the shape.
    // Note about naming: All objects in the shape are related to an entity in the geometry. Since all
    // geometry entities have a unique ID, these will be the "anchor" of the topology.
    void renewFromSketch(const Part::Part2DObject* sketch, const std::vector<Geometry*>& geometry);
    /// Create a face from the TopoShape's wires. For the sake of efficiency we assume that the TopoShape contains nothing except one or more wires
    void makeFace();

    /// Move the shape
    void move(const TopLoc_Location& m);

    /// Build a prism from the shape. Throws an exception if the TopoShape contains anything except a single face
    // Note: We use Standard_Real to test L for Precision::IsInfinite()
    void makePrism(const gp_Dir& dir, const Standard_Real L, const Standard_Real L2,
                   const bool midplane, const bool reversed);
    void makePrism(const TopoShape& base, const TopoDS_Face &supportface,
                   const gp_Dir &direction, const TopoDS_Shape &upToFace, const bool fuse = false);
    /// Build a revolution from the shape. Throws an exception if the TopoShape contains anything except a single face
    void makeRevolution(const gp_Ax1& axis, const Standard_Real angle, const bool midplane, const bool reversed);

    /// Fuse this TopoShape with another TopoShape. Returns only one solid even if multiple disconnected solids are produced
    void makeFuse(const TopoShape &other, const bool thisIsBase = true);
    /// Cut another TopoShape out of this TopoShape.  Returns only one solid even if multiple disconnected solids are produced
    void makeCut(const TopoShape& other, const bool thisIsBase = true);
    /// Find common material of this TopoShape and another TopoShape. Returns only one solid even if multiple disconnected solids are produced
    void makeCommon(const TopoShape &other);
    /// Section this TopoShape with another TopoShape. Returns only one solid even if multiple disconnected solids are produced
    void makeSection(const TopoShape &other);
    /// Compound this TopoShape with another TopoShape (any kind of geometry can be compounded)
    void makeCompound(const TopoShape &other);
    void makeCompound(const std::vector<TopoShape> &others);

    /// Chamfer the given edges on the TopoShape
    void makeChamfer(const std::vector<std::string>& edges, const Standard_Real size);
    /// Fillet the given edges on the TopoShape
    void makeFillet(const std::vector<std::string>& edges, const Standard_Real radius);
    /// Create draft on the given faces of the TopoShape
    void makeDraft(const std::vector<std::string>& faces, const gp_Dir& pullDirection,
                   const Standard_Real angle, const gp_Pln& neutralPlane);

    /// Transform the TopoShape
    void makeTransform(const gp_Trsf& tr);

    /// Model refine (joins coplanar faces)
    void refine();
    //@}

    TopoDS_Shape _Shape;
};

} //namespace Part


#endif // PART_TOPOSHAPE_H
