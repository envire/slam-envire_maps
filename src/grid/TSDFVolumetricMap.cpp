#include "TSDFVolumetricMap.hpp"
#include <boost/format.hpp>
#include <maps/tools/VoxelTraversal.hpp>

using namespace maps::grid;
using namespace maps::tools;

void TSDFVolumetricMap::mergePointCloud(const TSDFVolumetricMap::PointCloud& pc, const base::Transform3d& pc2grid, double measurement_variance)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin;

    // TODO add transformation uncertainty

    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        try
        {
            Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
            mergePoint(sensor_origin_in_grid, pc2grid * measurement, measurement_variance);
        }
        catch(const std::runtime_error& e)
        {
            // TODO use glog or base log for all out prints of this library
            std::cerr << e.what() << std::endl;
        }
    }
}

void TSDFVolumetricMap::mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement, double measurement_variance)
{
    Eigen::Vector3d truncated_direction = (measurement - sensor_origin).normalized();
    truncated_direction = truncation * truncated_direction;
    double ray_length = (measurement - sensor_origin).norm();
    Eigen::Vector3d start_point = sensor_origin;
    Eigen::Vector3d end_point = measurement + truncated_direction;

    Eigen::Vector3i start_point_idx;
    Eigen::Vector3i end_point_idx;
    Eigen::Vector3d start_point_cell_center;
    if(VoxelGridBase::toVoxelGrid(start_point, start_point_idx) &&
        VoxelGridBase::fromVoxelGrid(start_point_idx, start_point_cell_center) &&
        VoxelGridBase::toVoxelGrid(end_point, end_point_idx, false))
    {
        std::vector<VoxelTraversal::RayElement> ray;
        VoxelTraversal::computeRay(VoxelGridBase::getVoxelResolution(), start_point, start_point_idx, start_point_cell_center, end_point, end_point_idx, ray);

        if(ray.empty())
            throw std::runtime_error("Ray is empty!");

        // handle last cell in ray
        try
        {
            VoxelCellType& cell = getVoxelCell(end_point_idx);
            Eigen::Vector3d cell_center;
            if(fromVoxelGrid(end_point_idx, cell_center))
                cell.update(ray_length - (cell_center - sensor_origin).norm(), measurement_variance, truncation, min_varaince);
        }
        catch(const std::runtime_error& e)
        {
            // end point is out of grid, which is here an expected case
        }

        for(const VoxelTraversal::RayElement& element : ray)
        {
            try
            {
                DiscreteTree<VoxelCellType>& tree = GridMapBase::at(element.idx);
                Eigen::Vector3d cell_center;
                if(GridMapBase::fromGrid(element.idx, cell_center))
                {
                    int32_t z_end = element.z_last + element.z_step;
                    for(int32_t z_idx = element.z_first; z_idx != z_end; z_idx += element.z_step)
                    {
                        cell_center.z() = tree.getCellCenter(z_idx);
                        tree.getCellAt(z_idx).update(ray_length - (cell_center - sensor_origin).norm(), measurement_variance);
                    }
                }
                else
                {
                    std::cerr << "Failed to receive cell center of " << element.idx << " from grid." << std::endl;
                }
            }
            catch(const std::runtime_error& e)
            {
                //std::cerr << e.what() << std::endl;
                // rest of the ray is probably out of grid
                break;
            }
        }
    }
    else
        throw std::runtime_error((boost::format("Point %1% or is outside of the grid! Can't add to grid.") % measurement.transpose()).str());
}

bool TSDFVolumetricMap::hasSameFrame(const base::Transform3d& local_frame, const Vector2ui& num_cells, const Vector2d& resolution) const
{
     if(getResolution() == resolution && getNumCells() == num_cells && getLocalFrame().isApprox(local_frame))
         return true;
     return false;
}

float TSDFVolumetricMap::getTruncation()
{
    return truncation;
}

void TSDFVolumetricMap::setTruncation(float truncation)
{
    this->truncation = truncation;
}

float TSDFVolumetricMap::getMinVariance()
{
    return min_varaince;
}

void TSDFVolumetricMap::setMinVariance(float min_varaince)
{
    this->min_varaince = min_varaince;
}