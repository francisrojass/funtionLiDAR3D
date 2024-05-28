struct Punto
{
    double x, y;
};

// Scoring conos, ajustar altura dependiendo de LiDAR
float scoring(const pcl::PointXYZI cone_pos, const pcl::PointIndices cluster, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float r, float h, float t)
{
    /*
        * Copyright (c) 2024 Francisco José Rojas Ramirez and Álvaro Landero
        * Trabajo realizado en la Escuela Técnica Superior de Ingeniería Informática,
        * Universidad de Sevilla
    */
    
    /*
    r = cone_radius
    t = noise of lidar
    h = cone height
    */
    float sum = 0.0;
    float score;

    for (const auto &idx : cluster.indices)
    {
        const pcl::PointXYZI &p = cloud->at(idx);
        pcl::PointXYZI b;
        float distance = 0.0;
        b.x = cone_pos.x;
        b.y = cone_pos.y;
        b.z = -3.5; // Coger el punto mas bajo del cluster para la altura.

        if (p.z < (cone_pos.z + h))
        {
            Punto newP;
            newP.x = sqrt(abs((p.x - b.x) + (p.y - b.y)));
            newP.y = p.z - b.z;

            distance = abs((-(h / r) * newP.x) + (h - newP.y) / sqrt(pow(h / r, 2) + 1));
        }
        else
        {
            pcl::PointXYZI A;
            A.x = b.x;
            A.y = b.y;
            A.z = b.z + h;

            distance = pcl::euclideanDistance(A, p);
        }

        sum += 1.0 - std::min((distance * distance) / (t * t), 1.0f);
    }

    score = sum / cluster.indices.size();

    return score;
}
