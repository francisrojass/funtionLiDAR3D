// Función para ajustar un polinomio de grado 2
string fitQuadratic(const map<float, double> &data)
{
    int n = data.size();
    MatrixXd A(n, 3);
    VectorXd b(n);

    int i = 0;
    for (auto it = data.begin(); it != data.end(); ++it)
    {
        float x = it->first;
        double y = it->second;
        A(i, 0) = x * x;
        A(i, 1) = x;
        A(i, 2) = 1;
        b(i) = y;
        ++i;
    }

    Vector3d coeffs = A.colPivHouseholderQr().solve(b);

    if (coeffs(0) > 0) {
        return "amarillo";  // convexa
    } else {
        return "azul";      // cóncava
    }

    //return coeffs;
}

std::map<float, double> calcularIntensidadPromedioPorAnillo(const pcl::PointIndices cluster, pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr &velodyne_cloud, string &color)
{
    std::map<float, double> intensidadTotalPorAnillo;
    std::map<float, std::vector<float>> puntosPorAnillo;

    for (const auto &idx : cluster.indices)
    {
        const velodyne_pcl::PointXYZIRT &p = velodyne_cloud->at(idx);

        // Verificar si el anillo ya está en el mapa
        auto it = puntosPorAnillo.find(p.ring);

        if (it != puntosPorAnillo.end())
        {
            // El anillo ya existe en el mapa, agregar la intensidad al vector
            it->second.push_back(p.intensity);
        }
        else
        {
            // El anillo no existe en el mapa, agregarlo con la intensidad
            puntosPorAnillo[p.ring] = {p.intensity};
        }
    }
    // Calcular el promedio de intensidad por anillo y agregarlo al mapa intensidadTotalPorAnillo
    for (const auto &par : puntosPorAnillo)
    {
        float anillo = par.first;
        const std::vector<float> &intensidades = par.second;
        double sumaIntensidades = 0.0;
        for (const auto &intensidad : intensidades)
        {
            sumaIntensidades += intensidad;
        }
        double promedioIntensidades = sumaIntensidades / intensidades.size();
        intensidadTotalPorAnillo[anillo] = promedioIntensidades;
    }
    ///*
    // Imprimir los datos del mapa
    std::cout << "intensidadTotalPorAnillo:" << std::endl;
    for (const auto &pair : intensidadTotalPorAnillo)
    {
        std::cout << "Anillo: " << pair.first << ", Intensidad Total: " << pair.second << std::endl;
    }
    color = fitQuadratic(intensidadTotalPorAnillo);
    //*/
    return intensidadTotalPorAnillo;
}
