#include <vector>
#include "Point.hpp"


struct MetaData
{
    bool occupied;
    float min_z;
    float max_z;
};

struct PointUpdate
{
    int id;
    std::vector<Point> points;
    MetaData data;
};

class PointMapCell{
public:

    void AddUpdate(const int& update_id, const std::vector<Point>& points);
    void RemoveUpdate(const int& id);
    void UpdateMetaData();

    std::vector<PointUpdate> updates;
    MetaData meta_data;
};