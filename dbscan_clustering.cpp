#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>

void dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float eps, int minPts, std::vector<pcl::PointIndices> &cluster_indices)
{
    /*
        * Copyright (c) 2024 Francisco José Rojas Ramirez
        * Trabajo realizado en la Escuela Técnica Superior de Ingeniería Informática,
        * Universidad de Sevilla
    */
    
    std::vector<int> point_cluster_indices(cloud->points.size(), -1); // Para rastrear a qué clúster pertenece cada punto

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    int cluster_id = 0;

    for (std::size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (point_cluster_indices[i] != -1)
            continue;

        std::vector<int> neighbors;
        std::vector<float> distances;
        tree->radiusSearch(cloud->points[i], eps, neighbors, distances);

        if (neighbors.size() < minPts)
        {
            point_cluster_indices[i] = -2; // Marcar como ruido
            continue;
        }

        cluster_id++;
        pcl::PointIndices cluster;
        cluster.indices.push_back(i);
        point_cluster_indices[i] = cluster_id;

        std::set<int> search_queue(neighbors.begin(), neighbors.end());
        search_queue.erase(i);

        while (!search_queue.empty())
        {
            int idx = *search_queue.begin();
            search_queue.erase(search_queue.begin());

            if (point_cluster_indices[idx] == -2)
                point_cluster_indices[idx] = cluster_id;
            if (point_cluster_indices[idx] != -1)
                continue;

            cluster.indices.push_back(idx);
            point_cluster_indices[idx] = cluster_id;

            tree->radiusSearch(cloud->points[idx], eps, neighbors, distances);

            if (neighbors.size() >= minPts)
            {
                search_queue.insert(neighbors.begin(), neighbors.end());
            }
        }

        cluster_indices.push_back(cluster);
    }
}
