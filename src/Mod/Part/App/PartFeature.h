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


#ifndef PART_FEATURE_H
# include <TopTools_IndexedMapOfShape.hxx>
# include <TopAbs_ShapeEnum.hxx>
#define PART_FEATURE_H

#include "TopoShape.h"
#include "PropertyTopoShape.h"
#include <App/GeoFeature.h>
#include <App/FeaturePython.h>
#include <App/PropertyLinks.h>
#include <App/PropertyGeo.h>
// includes for findAllFacesCutBy()
#include <TopoDS_Face.hxx>
class gp_Dir;

class BRepBuilderAPI_MakeShape;

namespace Part { class BRepBuilderAPI_RefineModel; }

namespace Part
{

/// Struct for storing references to shapes
struct ShapeRef {
   /// The type of the shape being referenced
   TopAbs_ShapeEnum type;
   /// The OCC shape index
   Standard_Integer index;

   ShapeRef(const TopAbs_ShapeEnum t = TopAbs_SHAPE, const int idx = 0) : type(t), index(idx) {}

   inline const bool isEmpty() const { return type == TopAbs_SHAPE; }

   /// Create a reference from a subname, e.g. Face8
   ShapeRef(const std::string& subName);
   /// Return the reference as a subname
   const std::string toSubName() const;

   /// Compare two ShapeRefs (required for std::map keying)
   const bool operator<(const ShapeRef& other) const;

   /// Return the shape referenced
   inline const TopoDS_Shape toShape(const std::map<TopAbs_ShapeEnum, TopTools_IndexedMapOfShape*>& M)
    { return (*M.at(type))(index + 1); }
};

class PartFeaturePy;

/** Base class of all shape feature classes in FreeCAD
 */
class PartExport Feature : public App::GeoFeature
{
    PROPERTY_HEADER(Part::Feature);

public:
    /// Constructor
    Feature(void);
    virtual ~Feature();

    PropertyPartShape Shape;

    /** @name methods override feature */
    //@{
    /// recalculate the feature
    /// recompute only this object
    virtual App::DocumentObjectExecReturn *recompute(void);
    virtual App::DocumentObjectExecReturn *execute(void);
    virtual short mustExecute(void) const;
    //@}

    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const;

    virtual PyObject* getPyObject(void);
    virtual std::vector<PyObject *> getPySubObjects(const std::vector<std::string>&) const;

protected:
    void onChanged(const App::Property* prop);
    TopLoc_Location getLocation() const;
    /**
     * Build a history of changes
     * MakeShape: The operation that created the changes, e.g. BRepAlgoAPI_Common
     * type: The type of object we are interested in, e.g. TopAbs_FACE
     * newS: The new shape that was created by the operation
     * oldS: The original shape prior to the operation
     */
    ShapeHistory buildHistory(BRepBuilderAPI_MakeShape&, TopAbs_ShapeEnum type,
        const TopoDS_Shape& newS, const TopoDS_Shape& oldS);
    ShapeHistory joinHistory(const ShapeHistory&, const ShapeHistory&);

    /**
      * Some simple but imperfect topological naming
      * "Previous" and "Current" refers to the previous and current call of Feature::execute()
      * "Old" and "New" refers to the shape before and after the Feature::execute() call
      *   (e.g. the sketch and the support of a Pad would be the old shapes and the Pad itself the new shape)
      *
      *             | previous call | current call
      * ------------------------------------------
      * old feature | prevOld       | curOld
      * new feature | prevNew       | curNew
      *
      * What we need in order to update the LinkSub property is a mapping from prevNew to curNew. To build this map,
      * we first map prevNew to prevOld, then prevOld to curOld, and then curOld to curNew
      */
    /**
     * Build internal maps:
     * a) From prevNew to prevOld by inverting curOldToCurNew of the previous call
     * b) From curOld to curNew by using the information in the builder and the oldShapes
     * Multiple mapping paths are possible, e.g. for a boolean feature there are two old shapes (the base and the tool)
     * If the concatenate flag is set, then step a) is omitted, and the map obtained in step b) is
     * concatenated with the already existing map.
     * Example for the mapping paths from curOld to curNew for a Pad feature fused into support:
     * Path1: *Sketch* ->BRepPrimAPI_MakePrism-> *Prism* ->BRepAlgoAPI_Fuse-> *Pad*
     * Path2:                                  *Support* ->BRepAlgoAPI_Fuse-> *Pad*
     * In this case, the second call to buildMaps() needs to have the concatenate flag set
     */
    void buildMaps(BRepBuilderAPI_MakeShape* builder,
                   const std::vector<TopoDS_Shape>& oldShapes,
                   const bool concatenate = false);
    /// Specialization for BRepBuilderAPI_RefineModel because it can't be derived from BRepBuilderAPI_MakeShape
    void buildMaps(Part::BRepBuilderAPI_RefineModel* builder,
                   const std::vector<TopoDS_Shape>& oldShapes);
    /// Shortcut for a single makeShape operation, e.g. filleting
    void buildMaps(BRepBuilderAPI_MakeShape* mkShape, const TopoDS_Shape& oldShape);
    void buildMaps(Part::BRepBuilderAPI_RefineModel* mkShape, const TopoDS_Shape& oldShape);
    /// Shortcut for a double operation, e.g. boolean (two old shapes combine into one new shape)
    void buildMaps(BRepBuilderAPI_MakeShape* mkShape,
                   const TopoDS_Shape& oldShape1, const TopoDS_Shape& oldShape2,
                   const bool concatenate = false);

    /**
      * Remap all properties of this feature. buildMaps() must be called first to create prevNewToPrevOld and
      * curOldToCurNew. A map will be built from prevNew to curNew using these two maps above and the
      * prevNewToCurNew map of the oldFeatures. This map is then used to remap all LinkSub and LinkSubList properties
      * that reference this feature.
      */
    void remapProperties(const std::vector<const Part::Feature*>& oldFeatures);
    /// Shortcuts
    void remapProperties(const Part::Feature* oldFeature);
    void remapProperties(const Part::Feature* oldFeature1, const Part::Feature* oldFeature2);
    /// Specialization for SketchObject (there is no old shape)
    void remapProperties(const TopoDS_Shape& prevShape, const TopoDS_Shape& curShape);

private:
    /// Convert a previous element to the current element
    const ShapeRef convertPrevToCur(const ShapeRef& ref) const;
    inline const std::string convertPrevToCur(const std::string& sub) const
        { return convertPrevToCur(ShapeRef(sub)).toSubName(); }

    /// Update all relevant properties
    void updateProperties();
    /// Convert all elements in a property
    void updateProperty(App::PropertyLinkSub* prop);
    void updateProperty(App::PropertyLinkSubList *prop);

private:
    typedef std::vector<ShapeRef> RefVec;
    typedef std::map<ShapeRef, ShapeRef> RefMap;
    typedef std::map<ShapeRef, RefVec> RefMultiMap;
    /// Map prevNew to curNew
    RefMap prevToCurrent;
    /**
      * Map curOld to curNew
      * We need a vector because there can be several old shapes, e.g. sketch and support of a pad
      * We need a multimap because one old shape can create multiple new shapes, e.g. the edge of a sketch that
      * is used in a Pad will create two new edges and a face.
      */
    std::vector<RefMultiMap> curOldToCurNew;
    /// Map prevNew to prevOld
    std::vector<RefMap> prevNewToPrevOld;
};

class FilletBase : public Part::Feature
{
    PROPERTY_HEADER(Part::FilletBase);

public:
    FilletBase();

    App::PropertyLink   Base;
    PropertyFilletEdges Edges;

    short mustExecute() const;
};

typedef App::FeaturePythonT<Feature> FeaturePython;


/** Base class of all shape feature classes in FreeCAD
 */
class PartExport FeatureExt : public Feature
{
    PROPERTY_HEADER(Part::FeatureExt);

public:
    const char* getViewProviderName(void) const {
        return "PartGui::ViewProviderPartExt";
    }
};

// Utility methods
/**
 * Find all faces cut by a line through the centre of gravity of a given face
 * Useful for the "up to face" options to pocket or pad
 */
struct cutFaces {
    TopoDS_Face face;
    double distsq;
};

PartExport
std::vector<cutFaces> findAllFacesCutBy(const TopoDS_Shape& shape,
                                        const TopoDS_Shape& face, const gp_Dir& dir);

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
PartExport
const bool checkIntersection(const TopoDS_Shape& first, const TopoDS_Shape& second,
                             const bool quick, const bool touch_is_intersection);

} //namespace Part


#endif // PART_FEATURE_H

