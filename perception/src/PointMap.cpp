#include "PointMap.hpp"
#include <cmath>


PointMap::PointMap()
{
    x_size = 100;
    y_size = 100;
    SetNumCells(150, 150);
    update_counter = 0;
}
/*!
Returns the size of the x and y dimensions of the map
@return A pair representing the size of the map <X,Y>
*/ 
std::pair<double, double> PointMap::GetSize()
{
    return std::pair<double, double>(x_size, y_size);
}
/*!
Returns the size of each x/y cell in the map
@return A pair representing the cell interval/size <X,Y>
*/ 
std::pair<int, int> PointMap::GetNumCells()
{
    return std::pair<double, double>(x_num_cells, y_num_cells);
}

/*!
Returns the physical size of the cell of the map.
@return a std::pair representing <x,y> cell size of the map.
*/
std::pair<double, double> PointMap::GetCellSize()
{
    return std::pair<double, double>(x_cell_size, y_cell_size);
}

/*!
Sets the number of cells in the 2D Grid.
@param x_cells Number of horizontal cells. This value must be non-zero.
@param y_cells Number of vertical cells. This value must be non-zero.
*/
void PointMap::SetNumCells(const size_t& x_cells, const size_t& y_cells)
{
    if(x_cells <= 0 || y_cells <= 0)
        return;

    x_num_cells = x_cells;
    y_num_cells = y_cells;

    x_cell_size = x_size / (double)x_num_cells;
    y_cell_size = y_size / (double)y_num_cells;

    UpdateCellDimensions();
}

/*!
Sets the number of cells in the 2D Grid.
@param x size of the map in the x direction edge to edge.
@param y size of the map in the y direction edge to edge.
*/
void PointMap::SetSize(const double& x, const double& y)
{

    x_size = x;
    x_size = y;

    x_cell_size = x_size / (double)x_num_cells;
    y_cell_size = y_size / (double)y_num_cells;

    UpdateCellDimensions();
}

/*!
Adds a set of LIDAR points to the map. The function will return an ID
cooresponding to this set of points. The calling function shall call RemoveUpdate
before destroying these points as the map does not create a copy.
@see RemoveUpdate
@param points A vector of points to be added.
@return a "unique" identifier generated by a rolling counter which coorespondes to this 
update.
*/
int PointMap::AddLIDARPoints(const std::vector<Point>& points)
{
    update_counter++;

    int x_index, y_index;
    double x_min = x_size * -0.5;
    double y_min = y_size * -0.5;
    std::map< std::pair< int, int>, std::vector<Point> > map_update;
    
    for(auto point : points)
    {
        int x, y;
        
        x = int((point.x - x_min) / x_cell_size);
        y = int((point.y - y_min) / y_cell_size);

        if( x < 0 || x >= x_num_cells || 
            y < 0 || y >= y_num_cells ||
            point.z >= z_cut_off)
            continue;

        map_update[std::pair< int, int>(x,y)].push_back(point);
    }

    for( auto it : map_update )
    {
        cell_map[it.first].AddUpdate(update_counter, it.second);
    }

    UpdateCellDimensions();
    return update_counter;
}

/*!
Removes an update (i.e. a set of points previously provided) from the map.
@see AddLIDARPoints
@param ID The identifier cooresponding to the returned ID when the points were pushed to the map
*/
void PointMap::RemoveUpdate(int ID)
{
    for( auto cell : cell_map )
    {
        cell.second.RemoveUpdate(ID);
    }
}

/*!
Clears the entire point map of points.
*/
void PointMap::ClearMap()
{
    cell_map.clear();
}

void PointMap::UpdateCellDimensions()
{
    const double map_x_min = (x_num_cells * x_cell_size) / -2;
    const double map_y_min = (y_num_cells * y_cell_size) / -2;
    
    std::map< std::pair< int, int> , PointMapCell>::iterator it = cell_map.begin();
    while( it != cell_map.end() )
    {
        PointMapCell& cell = it->second;
        const std::pair< int, int>& index = it->first;
        const int& x_index = index.first;
        const int& y_index = index.second;
        cell.SetSize(x_cell_size, y_cell_size);

        std::pair< double, double > left_edge_point, right_edge_point;

        //think about looking at a cube from a left angle and a right angle. If looking from the left.
        //You need these points to find the window..
        //   *
        //  / [ ]
        // /     *
        //      /
        //From the right you need these.
        //      *
        //   [ ] \
        //  *     \
        //   \
        //that's what is going on here in the indexing.
        if(index.first < x_num_cells / 2)
        {
            left_edge_point.first = map_x_min + (x_index * x_cell_size);
            left_edge_point.second = map_y_min + (y_index * y_cell_size);

            right_edge_point.first = map_x_min + ((x_index+1) * x_cell_size);
            right_edge_point.second = map_y_min + ((y_index+1) * y_cell_size);
        }
        else
        {
            left_edge_point.first = map_x_min + (x_index * x_cell_size);
            left_edge_point.second = map_y_min + ((y_index+1) * y_cell_size);

            right_edge_point.first = map_x_min + ((x_index+1) * x_cell_size);
            right_edge_point.second = map_y_min + (y_index * y_cell_size);
        }
        double a1 = atan(left_edge_point.second / left_edge_point.first);
        double a2 = atan(right_edge_point.second / right_edge_point.first);
        double r = sqrt( pow((left_edge_point.first + right_edge_point.first)/2, 2) + 
                         pow((left_edge_point.second + right_edge_point.second)/2, 2));

        cell.a1 = a1 * -1;
        cell.a2 = a2 * -1;
        cell.SetDistance(r);
        it++;
    }
}