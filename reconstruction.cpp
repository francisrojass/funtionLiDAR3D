void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_f, std::vector<pcl::PointXYZI> punto)
{
    /*
        * Copyright (c) 2024 Francisco José Rojas Ramirez
        * Trabajo realizado en la Escuela Técnica Superior de Ingeniería Informática,
        * Universidad de Sevilla
    */
    
    // Crear el objeto KdTreeFLANN una sola vez fuera del bucle
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_f);

    // Definir el radio de búsqueda
    float search_radius = 0.20; // Puedes ajustar este valor segun sea necesario

    // Desenrollar el bucle externo
    for (size_t idx = 0; idx < punto.size(); ++idx) // Incrementar en 2 en cada iteración
    {
        // Buscar los puntos cercanos a los puntos medio y siguiente
        std::vector<int> point_indices;
        std::vector<float> point_distances;

        // Verificar si se encuentran puntos dentro del radio de búsqueda alrededor de los puntos medio y siguiente
        if (kdtree.radiusSearch(punto[idx], search_radius, point_indices, point_distances) > 0)
        {
            // Iterar sobre los índices de los puntos encontrados
            for (size_t i = 0; i < point_indices.size(); ++i)
            {
                int point_index = point_indices[i];
                cloud->emplace_back(cloud_f->points[point_index]);
            }
        }
    }
}

//En vez de recostruir la nube, recostruye los cluster para despues usarlos en el mapping
void recostruccion1(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_f, 
    const std::vector<pcl::PointXYZI> &puntos,
    std::vector<pcl::PointIndices>& cluster_indices) 
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_f);
    float search_radius = 0.15;

    for (size_t idx = 0; idx < puntos.size(); ++idx) {
        std::vector<int> point_indices;
        std::vector<float> point_distances;

        if (kdtree.radiusSearch(puntos[idx], search_radius, point_indices, point_distances) > 0) {
            for (size_t i = 0; i < point_indices.size(); ++i) {
                int point_index = point_indices[i];

                // Aquí seleccionamos el cluster al que queremos añadir el punto
                // (Puedes cambiar esta lógica para seleccionar el cluster correcto)
                int cluster_to_modify = idx; // Ejemplo: siempre añadir al primer cluster

                // Añadir el índice del punto al cluster seleccionado
                cluster_indices[cluster_to_modify].indices.push_back(point_index);
            }
        }
    }
}
