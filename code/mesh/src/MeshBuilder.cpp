#include <algorithm>
#include <iostream>

#include "MeshBuilder.hpp"
#include "mesh_manipulations.hpp"

Eigen::Matrix<double,3,Eigen::Dynamic> MeshBuilder::get_nodes() const
{
    return nodes;
}

std::vector<Facet> MeshBuilder::get_facets() const
{
    return facets;
}

MeshBuilder::MeshBuilder(const VectorOfVectorOfPoints& v_) : v(v_),
                                                                   xyzMap(Vector3dMap()),
                                                                   index(0),
                                                                   nodes(Eigen::Matrix<double,3,Eigen::Dynamic>()),
                                                                   facets(std::vector<Facet>())
{
    if (not(v.empty())) nodes = Eigen::MatrixXd::Zero(3,(int)(v.size()*v.front().size()));
}

MeshBuilder::MeshBuilder(const VectorOfPoints& tri) : v(VectorOfVectorOfPoints(1,tri)),
                                                            xyzMap(Vector3dMap()),
                                                            index(0),
                                                            nodes(Eigen::Matrix<double,3,Eigen::Dynamic>()),
                                                            facets(std::vector<Facet>())
{
    if (not(tri.empty())) nodes = Eigen::MatrixXd::Zero(3,(int)(v.size()*v.front().size()));
}

Eigen::Matrix<double,3,Eigen::Dynamic> MeshBuilder::resize(const Eigen::Matrix<double,3,Eigen::Dynamic>& M) const
{
    Eigen::Matrix<double,3,Eigen::Dynamic> resized(Eigen::MatrixXd::Zero(3,(int)index));
    for (size_t j = 0 ; j < index ; ++j)
    {
        resized.col((int)j) = M.col((int)j);
    }
    return resized;
}

Mesh MeshBuilder::build()
{
    *this = std::for_each(v.begin(), v.end(), *this);
    return Mesh(resize(nodes), facets);
}

void MeshBuilder::operator()(const VectorOfPoints& list_of_points)
{
    if (not(list_of_points.empty()))
    {
        Facet facet;
        const Matrix3x M = convert(list_of_points);
        facet.unit_normal = unit_normal(M);
        facet.area = area(M);
        facet.barycenter = barycenter(M);
        for (VectorOfPoints::const_iterator it = list_of_points.begin() ; it != list_of_points.end() ; ++it)
        {
            facet.index.push_back(build_one_point(*it));
        }
        facets.push_back(facet);
    }
}

size_t MeshBuilder::build_one_point(const EPoint& xyz)
{
    const bool point_has_been_added = add_point_if_missing(xyz);
    if (point_has_been_added) index++;
    return xyzMap[xyz];
}

bool MeshBuilder::add_point_if_missing(const EPoint& xyz)
{
    bool point_has_been_added = false;
    if (not(point_is_in_map(xyz)))
    {
        xyzMap.insert(std::make_pair(xyz,index));
        nodes.col((int)index) = xyz;
        point_has_been_added = true;
    }
    return point_has_been_added;
}

bool MeshBuilder::point_is_in_map(const EPoint& xyz)
{
    const Vector3dMap::const_iterator itMap = xyzMap.find(xyz);
    return itMap != xyzMap.end();
}
