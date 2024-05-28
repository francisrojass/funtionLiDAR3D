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

// Función para procesar los datos del LiDAR
void processLidarData(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground_cloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &remaining_cloud, int n_segments)
{

    /*
        * Copyright (c) 2024 Francisco José Rojas Ramirez
        * Trabajo realizado en la Escuela Técnica Superior de Ingeniería Informática,
        * Universidad de Sevilla
    */
  
    // Ajustar la longitud de los datos
    cloud->points.resize(cloud->points.size() - (cloud->points.size() % n_segments));

    // Dividir los datos en segmentos
    std::vector<std::vector<pcl::PointXYZI>> separated_data;
    for (int i = 0; i < n_segments; ++i)
    {
        separated_data.push_back(std::vector<pcl::PointXYZI>(cloud->points.begin() + i * (cloud->points.size() / n_segments),
                                                             cloud->points.begin() + (i + 1) * (cloud->points.size() / n_segments)));
    }

    // Procesamiento de los datos por segmento
    std::unordered_map<std::pair<int, int>, float, pair_hash> min_z;
    std::vector<bool> is_ground;
    for (int i = 0; i < n_segments; ++i)
    {
        for (const auto &p : separated_data[i])
        {
            int d = 0;
            if (p.x > -100)
            {
                d = static_cast<int>(std::sqrt(p.x * p.x + p.y * p.y));
            }
            auto key = std::make_pair(i, d);
            if (min_z.find(key) != min_z.end())
            {
                min_z[key] = std::min(p.z, min_z[key]);
                if (p.z < min_z[key] + 0.05)
                {
                    is_ground.push_back(true);
                }
                else
                {
                    is_ground.push_back(false);
                }
            }
            else
            {
                min_z[key] = p.z;
                is_ground.push_back(true);
            }
        }
    }

    // Selección de los puntos del suelo y los puntos restantes
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (is_ground[i])
        {
            ground_cloud->points.push_back(cloud->points[i]); // Agregar el punto al suelo
        }
        else
        {
            remaining_cloud->points.push_back(cloud->points[i]); // Agregar el punto a los puntos restantes
        }
    }

    // Ahora tienes dos nuevas nubes de puntos: ground_cloud para los puntos del suelo y remaining_cloud para los puntos restantes
}
