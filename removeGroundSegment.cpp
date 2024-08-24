struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};

void filter_segments(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     pcl::PointIndices::Ptr &inliers,
                     pcl::PointIndices::Ptr &outliers,
                     int n_segments,
                     float threshold)
{
    const float PI = 3.14159265358979323846;
    std::unordered_map<std::pair<int, int>, float, pair_hash> min_z_map;

    for (std::size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZI p = cloud->at(i);

        float angle = atan2(p.y, p.x); // Usar atan2 para obtener el ángulo correcto
        if (angle < 0)
        {
            angle += 2 * PI; // Asegurarse de que el ángulo esté en [0, 2*PI)
        }

        int segment = static_cast<int>(angle / (2 * PI / n_segments));
        int distance = static_cast<int>(std::sqrt(p.x * p.x + p.y * p.y));
        std::pair<int, int> key = {segment, distance};

        if (min_z_map.find(key) == min_z_map.end())
        {
            // Si no existe el segmento, agregarlo y mantener el punto actual
            min_z_map[key] = p.z;
            inliers->indices.push_back(i);
        }
        else
        {
            if (p.z >= min_z_map[key] + threshold)
            {
                // Si el valor de z es mayor que el valor mínimo más el umbral, mantener el punto actual
                outliers->indices.push_back(i);
            }
            else
            {
                // Si el valor de z es menor, actualizar el mínimo z y agregar el punto actual a inliers
                min_z_map[key] = p.z;
                inliers->indices.push_back(i);
            }
        }
    }
}
