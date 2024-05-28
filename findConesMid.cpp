struct PolarPoint
{
    float radius;
    float angle;
};

// Función para convertir un punto de coordenadas cartesianas a polares
PolarPoint cartesianToPolar(const pcl::PointXYZI &point)
{
    PolarPoint polar_point;
    polar_point.radius = sqrt(point.x * point.x + point.y * point.y);
    polar_point.angle = atan2(point.y, point.x);
    return polar_point;
}

// Función para encontrar los puntos medios de los conos en un cluster
std::vector<pcl::PointXYZI> findConesMidpoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<pcl::PointXYZI> puntos_medios; // Vector para almacenar los puntos medios

    for (const auto &indices : cluster_indices)
    {
        std::vector<PolarPoint> polar_points;
        for (const auto &index : indices.indices)
        {
            const pcl::PointXYZI &point = cloud->points[index];
            polar_points.push_back(cartesianToPolar(point));
        }

        std::sort(polar_points.begin(), polar_points.end(),
                  [](const PolarPoint &p1, const PolarPoint &p2)
                  {
                      return p1.angle > p2.angle;
                  });

        pcl::PointXYZI p1, p2, midpoint;
        p1.x = polar_points[0].radius * cosf(polar_points[0].angle);
        p1.y = polar_points[0].radius * sinf(polar_points[0].angle);
        p1.z = 0.f;
        p1.intensity = 0.f;

        p2.x = polar_points[1].radius * cosf(polar_points[1].angle);
        p2.y = polar_points[1].radius * sinf(polar_points[1].angle);
        p2.z = 0.f;
        p2.intensity = 0.f;

        midpoint.x = (p1.x + p2.x) / 2.;
        midpoint.y = (p1.y + p2.y) / 2.;
        midpoint.z = (p1.z + p2.z) / 2.;
        midpoint.intensity = 0.0;

        puntos_medios.push_back(midpoint); // Agregar el punto medio al vector
    }

    return puntos_medios;
}
