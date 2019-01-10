#include <path_follower/follower.h>

Follower::Follower() {
    nh = ros::NodeHandle();

    // Subscriptions
    sub_pose = nh.subscribe("/uav_1/ual/pose", 10, &Follower::UALPoseCallback, this);
    sub_state = nh.subscribe("/uav_1/ual/state", 10, &Follower::UALStateCallback, this);
    sub_current_velocity = nh.subscribe("/uav_1/mavros/local_position/velocity", 10, &Follower::UALVelocityCallback, this);
    sub_velocity = nh.subscribe("velSpline", 10, &Follower::UALPathVCallback, this);
    sub_path = nh.subscribe("output_path", 10, &Follower::UALPathCallback, this);
    sub_new_vectorT = nh.subscribe("output_path", 10, &Follower::newVectorTCallback, this);

    // Publishers
    pub_to_target = nh.advertise<nav_msgs::Path>("distToTarget", 1000);
    pub_normal_distance = nh.advertise<nav_msgs::Path>("distNormal", 1000);
    pub_look_ahead = nh.advertise<nav_msgs::Path>("distLookAhead", 1000);
    pub_draw_current_path = nh.advertise<nav_msgs::Path>("drawActualPath", 1000);
    pub_visualization_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    pub_set_pose = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/ual/set_pose", 1000);
    pub_set_velocity = nh.advertise<geometry_msgs::TwistStamped>("/uav_1/ual/set_velocity", 1000);

    // Service
    srvTakeOff = nh.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
    srvLand = nh.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");
    srvGoToWaypoint = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("/uav_1/ual/go_to_waypoint");
    srvSetVelocity = nh.serviceClient<uav_abstraction_layer::SetVelocity>("/uav_1/ual/set_velocity");

    mision();
}

Follower::~Follower() {
}

// Moving Average
using ecl::FiFo;
class movingAvg {
   public:
    movingAvg(const unsigned int widowsSize) : sum(0.0), average(0.0), windows_size(widowsSize), first(true) {
        fifo.resize(windows_size);
    }
    void reset() {
        fifo.fill(0);
        sum = 0;
        average = 0.0;
    }
    void update(const double &incomingData) {
        if (first) {
            fifo.fill(incomingData);
            first = false;
        }

        sum -= fifo[0];
        sum += incomingData;
        fifo.push_back(incomingData);
        average = sum / (double)(windows_size);
    }
    double sum;
    double average;
    unsigned int windows_size;
    FiFo<double> fifo;
    bool first;
};

void Follower::purePursuit() {
    float comp_dist_resta;
    std::vector<float> comp_dist;
    std::vector<float>::iterator up;
    // Obtiene la magnitud de la distancia normal.
    normal_distance = calculateNormalDistance();
    // Obtiene la posición en el path de la distancia normal.
    pos_path = calculateNormalDistancePos();
    // Determinar que waypoint está a una distancia similar a la distancia look ahead,
    // respecto del waypoint que corresponde con la normal del dron en este instante.
    // if (normal_distance < look_ahead)
    // {
    for (pos_path; pos_path < path_ok.size() - 1; pos_path++) {
        comp_dist_resta = distance2PointsDirection(path_ok[pos_path].pose.position.x,
                                                   path_ok[pos_path].pose.position.y,
                                                   path_ok[pos_path].pose.position.z, look_ahead);

        comp_dist.push_back(comp_dist_resta);
    }
    up = std::upper_bound(comp_dist.begin(), comp_dist.end(), look_ahead);
    pure_pursuit_pos = up - comp_dist.begin();
    // Se actualiza la distancia look ahead en funcion de como evoluciona la distancia
    // normal en el timepo.
    // changeLookAheadVariable(); // Descomentar para tener el generador V1
    // Si el dron está lejos del path, se le da la orden de que se aproxime por la
    // distancia mas corta.
    // }
    // else if (normal_distance > look_ahead /* && pure_pursuit_pos > 0 */)
    // {
    //     pure_pursuit_pos = pos_path;
    //     flag_pure_pursuit = false;
    //     std::cout << "[ TEST] Going Closer" << '\n';
    // }
    // Limpia el vector
    comp_dist.clear();
}

void Follower::changeLookAhead(int p) {
    // Si el look_ahead del estado anterior es diferente que el waypoint objetivo,
    // suma o resta un porcentaje para suavizar las aceleraciones.
    /*if 			 	((1/new_vectorT[p]) !=  prev_look_ahead){
		if      ((1/new_vectorT[p]) > prev_look_ahead){
			look_ahead = prev_look_ahead + 1 * 0.001;
		}else if((1/new_vectorT[p]) < prev_look_ahead){
			look_ahead = prev_look_ahead - 1 * 0.001;
		}
	}else if  ((1/new_vectorT[p]) == prev_look_ahead){
		look_ahead = 1/new_vectorT[p];
	}
	prev_look_ahead = look_ahead;*/
    float target_x = path_ok[p].pose.position.x;
    float target_y = path_ok[p].pose.position.y;
    float target_z = path_ok[p].pose.position.z;
    to_target_distance = distance2Points(target_x, current_x, target_y, current_y, target_z, current_z);
    // look_ahead = 1 / new_vectorT[p]; // normal
    look_ahead = 1; // Try manager simple
    // std::cout << "[ TEST] Look Ahead = " << look_ahead << " | Target = " << 1 / new_vectorT[p] << " | P = " << p << '\n';
}

void Follower::changeLookAheadVariable() {
    normal_distance = calculateNormalDistance();
    // Si el dron está lejos del path, la distancia look ahead toma el valor de la distancia
    // normal en ese momento.
    if (normal_distance > init_look_ahead * 3) {
        look_ahead = look_ahead - init_look_ahead * 0.1;
        // Si la distancia normal disminuye respecto del instante anterior y el dron está cerca del path
        // aumenta la distancia look ahead un 10% de la distancia look ahead inicial.
    } else if (normal_distance < prev_normal_distance * 0.95 && look_ahead < init_look_ahead * 3) {
        look_ahead = look_ahead + init_look_ahead * 0.1;
        // Si la distancia normal aumenta respecto del instante anterior y el dron está cerca del path
        // disminuye la distancia look ahead un 10% de la distancia look ahead inicial.
    } else if (normal_distance > prev_normal_distance * 1.05 && look_ahead > init_look_ahead * 1.05) {
        look_ahead = look_ahead - init_look_ahead * 0.1;
    }
    // Guarda el valor actual de la distancia normal, para poder utilizarlo en el siguiente instante
    // como comprobación.
    prev_normal_distance = normal_distance;
}

float Follower::distance2Points(float x1, float x2, float y1, float y2, float z1, float z2) {
    float mod;
    mod = sqrt((x2 - x1) * (x2 - x1) +
               (y2 - y1) * (y2 - y1) +
               (z2 - z1) * (z2 - z1));
    return mod;
}

float Follower::distance2PointsDirection(float x1, float y1, float z1, float LA) {
    float mod;
    float x2, y2, z2;
    // Calcula el módulo teniendo en cuenta el signo del waypoint para poder sumar
    // la distancia look ahead correctamente.
    if (x1 < 0) {
        x2 = x1 - LA;
    }
    if (x1 > 0) {
        x2 = x1 + LA;
    }
    if (y1 < 0) {
        y2 = y1 - LA;
    }
    if (y1 > 0) {
        y2 = y1 + LA;
    }
    if (z1 < 0) {
        z2 = z1 - LA;
    }
    if (z1 > 0) {
        z2 = z1 + LA;
    }

    mod = distance2Points(x1, x2, y1, y2, z1, z2);

    return mod;
}

float Follower::calculateNormalDistance() {
    std::vector<float> smallest_dist;
    float smallest_dist_sqrt;
    for (int x = 0; x < path_ok.size() - 1; x++) {
        smallest_dist_sqrt = distance2Points(path_ok[x].pose.position.x, current_x,
                                             path_ok[x].pose.position.y, current_y,
                                             path_ok[x].pose.position.z, current_z);
        smallest_dist.push_back(smallest_dist_sqrt);
    }
    auto smallest_dist_min = std::min_element(smallest_dist.begin(), smallest_dist.end());
    // Devuelve la magnitud de la distancia normal
    return *smallest_dist_min;
}

float Follower::calculateNormalDistancePos() {
    std::vector<float> smallest_dist;
    float smallest_dist_sqrt;
    for (int x = 0; x < path_ok.size() - 1; x++) {
        smallest_dist_sqrt = distance2Points(path_ok[x].pose.position.x, current_x,
                                             path_ok[x].pose.position.y, current_y,
                                             path_ok[x].pose.position.z, current_z);
        smallest_dist.push_back(smallest_dist_sqrt);
    }
    auto smallest_dist_min = std::min_element(smallest_dist.begin(), smallest_dist.end());
    auto comp_dist_pos = smallest_dist_min - smallest_dist.begin();
    // Devuelve la posición en el path de la distancia normal
    return comp_dist_pos;
}

float Follower::calculateSumNormalDistance() {
    sum_normal_distance = sum_normal_distance + calculateNormalDistance();
    return sum_normal_distance;
}

void Follower::drawCylinder() {
    marker.header.frame_id = "uav_1_home";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_x;
    marker.pose.position.y = current_y;
    marker.pose.position.z = current_z - 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.6;
    marker.color.a = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
}

void Follower::drawTriangles(int p) {
    std::vector<grvc::ual::Waypoint> wpList_distToTarget;
    std::vector<grvc::ual::Waypoint> wpList_distNormal;
    std::vector<grvc::ual::Waypoint> wpList_look_ahead;
    grvc::ual::Waypoint wp_distToTarget;
    grvc::ual::Waypoint wp_distNormal;
    grvc::ual::Waypoint wp_look_ahead;

    wp_distToTarget.pose.position.x = current_x;
    wp_distToTarget.pose.position.y = current_y;
    wp_distToTarget.pose.position.z = current_z;
    wpList_distToTarget.push_back(wp_distToTarget);
    wp_distToTarget.pose.position.x = path_ok[p].pose.position.x;
    wp_distToTarget.pose.position.y = path_ok[p].pose.position.y;
    wp_distToTarget.pose.position.z = path_ok[p].pose.position.z;
    wpList_distToTarget.push_back(wp_distToTarget);
    std::vector<geometry_msgs::PoseStamped> posesToTarget(wpList_distToTarget.size());
    for (int x = 0; x < wpList_distToTarget.size(); x++) {
        posesToTarget.at(x).pose.position.x = wpList_distToTarget[x].pose.position.x;
        posesToTarget.at(x).pose.position.y = wpList_distToTarget[x].pose.position.y;
        posesToTarget.at(x).pose.position.z = wpList_distToTarget[x].pose.position.z;
    }
    path_to_target_distance.header.frame_id = "uav_1_home";
    path_to_target_distance.poses = posesToTarget;

    pos_path = calculateNormalDistancePos();

    wp_distNormal.pose.position.x = current_x;
    wp_distNormal.pose.position.y = current_y;
    wp_distNormal.pose.position.z = current_z;
    wpList_distNormal.push_back(wp_distNormal);
    wp_distNormal.pose.position.x = path_ok[pos_path].pose.position.x;
    wp_distNormal.pose.position.y = path_ok[pos_path].pose.position.y;
    wp_distNormal.pose.position.z = path_ok[pos_path].pose.position.z;
    wpList_distNormal.push_back(wp_distNormal);
    std::vector<geometry_msgs::PoseStamped> posesNormal(wpList_distNormal.size());
    for (int x = 0; x < wpList_distNormal.size(); x++) {
        posesNormal.at(x).pose.position.x = wpList_distNormal[x].pose.position.x;
        posesNormal.at(x).pose.position.y = wpList_distNormal[x].pose.position.y;
        posesNormal.at(x).pose.position.z = wpList_distNormal[x].pose.position.z;
    }
    path_normal_distance.header.frame_id = "uav_1_home";
    path_normal_distance.poses = posesNormal;

    wp_look_ahead.pose.position.x = path_ok[p].pose.position.x;
    wp_look_ahead.pose.position.y = path_ok[p].pose.position.y;
    wp_look_ahead.pose.position.z = path_ok[p].pose.position.z;
    wpList_look_ahead.push_back(wp_look_ahead);
    wp_look_ahead.pose.position.x = path_ok[pos_path].pose.position.x;
    wp_look_ahead.pose.position.y = path_ok[pos_path].pose.position.y;
    wp_look_ahead.pose.position.z = path_ok[pos_path].pose.position.z;
    wpList_look_ahead.push_back(wp_look_ahead);
    std::vector<geometry_msgs::PoseStamped> posesLookAhead(wpList_look_ahead.size());
    for (int x = 0; x < wpList_look_ahead.size(); x++) {
        posesLookAhead.at(x).pose.position.x = wpList_look_ahead[x].pose.position.x;
        posesLookAhead.at(x).pose.position.y = wpList_look_ahead[x].pose.position.y;
        posesLookAhead.at(x).pose.position.z = wpList_look_ahead[x].pose.position.z;
    }
    path_look_ahead.header.frame_id = "uav_1_home";
    path_look_ahead.poses = posesLookAhead;
}

void Follower::currentTrajectory() {
    grvc::ual::Waypoint waypoint;
    // Se guarda en un vector auxiliar la posicion de cada instante
    waypoint.pose.position.x = current_x;
    waypoint.pose.position.y = current_y;
    waypoint.pose.position.z = current_z;
    aux_vector.push_back(waypoint);
    // Se crea el vector de la trayectoria actual
    std::vector<geometry_msgs::PoseStamped> posesActual(i + 1);
    msg_current_draw.header.frame_id = "uav_1_home";
    for (int j = 0; j < posesActual.size() - 1; j++) {
        // Se coloca en la trayectoria actual, las posiciones seguidas con anterioridad
        posesActual.at(j).pose.position.x = aux_vector.at(j).pose.position.x;
        posesActual.at(j).pose.position.y = aux_vector.at(j).pose.position.y;
        posesActual.at(j).pose.position.z = aux_vector.at(j).pose.position.z;
    }
    // Se coloca en la trayectoria actual, la posicion actual
    posesActual.at(i).pose.position.x = current_x;
    posesActual.at(i).pose.position.y = current_y;
    posesActual.at(i).pose.position.z = current_z;

    msg_current_draw.poses = posesActual;
}

void Follower::UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_x = msg->pose.position.x;
    current_y = msg->pose.position.y;
    current_z = msg->pose.position.z;

    return;
}

void Follower::UALStateCallback(const uav_abstraction_layer::State &msg) {
    state = msg.state;

    return;
}

void Follower::UALVelocityCallback(const geometry_msgs::TwistStamped &msg) {
    current_velocity_x = msg.twist.linear.x;
    current_velocity_y = msg.twist.linear.y;
    current_velocity_z = msg.twist.linear.z;

    return;
}

void Follower::UALPathCallback(const nav_msgs::Path &msg) {
    geometry_msgs::PoseStamped waypoint;
    if (flag_sub_path == true) {
        for (int p = 0; p < msg.poses.size(); p++) {
            waypoint.header.frame_id = msg.header.frame_id;
            waypoint.pose.position.x = msg.poses.at(p).pose.position.x;
            waypoint.pose.position.y = msg.poses.at(p).pose.position.y;
            waypoint.pose.position.z = msg.poses.at(p).pose.position.z;
            waypoint.pose.orientation.x = msg.poses.at(p).pose.orientation.x;
            waypoint.pose.orientation.y = msg.poses.at(p).pose.orientation.y;
            waypoint.pose.orientation.z = msg.poses.at(p).pose.orientation.z;
            waypoint.pose.orientation.w = msg.poses.at(p).pose.orientation.w;
            path_ok.push_back(waypoint);
        }
    }
    // ROS_WARN("path 0 x: %f", path_ok[0].pose.position.x);
    flag_sub_path = false;

    return;
}

void Follower::UALPathVCallback(const nav_msgs::Path &msg) {
    grvc::ual::Velocity v;
    if (flag_sub_velocity == true) {
        for (int p = 0; p < msg.poses.size(); p++) {
            v.twist.linear.x = msg.poses.at(p).pose.position.x;
            v.twist.linear.y = msg.poses.at(p).pose.position.y;
            v.twist.linear.z = msg.poses.at(p).pose.position.z;
            path_v.push_back(v);
        }
    }
    flag_sub_velocity = false;

    return;
}

void Follower::newVectorTCallback(const nav_msgs::Path &msg) {
    if (flag_sub_vectorT == true) {
        for (int p = 0; p < msg.poses.size(); p++) {
            new_vectorT.push_back(1);
        }
    }
    flag_sub_vectorT = false;
    return;
}

void Follower::mision() {
    while (state != 2 && ros::ok()) {
        ros::spinOnce();
        sleep(1);
    }
    // Despega el dron
    // ual.takeOff(flight_level, true);
    take_off.request.height = 5.0;
    take_off.request.blocking = true;
    srvTakeOff.call(take_off);
    // Lleva el dron al waypoint inicial
    path_ok[0].pose.orientation.x = 0;
    path_ok[0].pose.orientation.y = 0;
    path_ok[0].pose.orientation.z = 0;
    path_ok[0].pose.orientation.w = 1;
    // go_to_waypoint.request.waypoint = path_ok[0];
    // go_to_waypoint.request.blocking = true;
    // srvGoToWaypoint.call(go_to_waypoint);
    pub_set_pose.publish(path_ok[0]);

    // ual.goToWaypoint(path_ok[0], true);
    // Inicializa Moving Average
    int windowsAvg = 3;
    movingAvg mavgVx(windowsAvg);
    movingAvg mavgVy(windowsAvg);
    movingAvg mavgVz(windowsAvg);
    for (int i = 0; i < windowsAvg; i++) {
        mavgVx.update(0);
        mavgVy.update(0);
        mavgVz.update(0);
    }
    // Espera en el waypoint inicial para tratar de empezar con velocidad 0
    ros::Duration(10).sleep();
    // Empieza a contar el tiempo para saber cuanto tarda en realizar la misión
    ros::Time begin = ros::Time::now();
    // ---------------------------------------------- MISION ----------------------------------------------
    std::cout << "[ TEST] Start mission!" << '\n';
    float target_x, target_y, target_z;
    float cont = 0;
    // Abre el fichero .dat
    std::ofstream fileVelocity, filePosition, fileMovingAvg, fileActualLA, fileT;
    fileVelocity.open("/home/hector/Matlab_ws/TFM/velocity.dat");
    filePosition.open("/home/hector/Matlab_ws/TFM/position.dat");
    fileMovingAvg.open("/home/hector/Matlab_ws/TFM/movingAvg.dat");
    fileActualLA.open("/home/hector/Matlab_ws/TFM/actualLA.dat");
    fileT.open("/home/hector/Matlab_ws/TFM/t.dat");
    // Recorre la trayectoria/path de waypoints
    for (int p = 0; p < path_ok.size(); p++) {
        // Determina el look ahead de este instante en funcion del tiempo que tiene
        // el dron para llegar al waypoint objetivo.
        changeLookAhead(p);  // Comentar para tener el generador V1
        // Determina la distancia normal
        normal_distance = calculateNormalDistance();
        // Determina cual es el waypoint objetivo.
        purePursuit();
        p = p + pure_pursuit_pos;
        // Asignación de variables del waypoint objetivo.
        target_x = path_ok[p].pose.position.x;
        target_y = path_ok[p].pose.position.y;
        target_z = path_ok[p].pose.position.z;
        to_target_distance = distance2Points(target_x, current_x, target_y, current_y, target_z, current_z);
        // Publica el triangulos que se dibujan en rviz, donde los lados son: la distancia normal,
        // la distancia look ahead y la distancia hasta el objetivo.
        drawCylinder();
        drawTriangles(p);
        ros::spinOnce();
        // El dron está dentro del path. Se le otorga la velocidad necesaria para que vaya hacia
        // el waypoint objetivo.
        while (normal_distance < look_ahead && to_target_distance > look_ahead) {
            normal_distance = calculateNormalDistance();
            to_target_distance = distance2Points(target_x, current_x, target_y, current_y, target_z, current_z);
            mavgVx.update(target_x - current_x);
            mavgVy.update(target_y - current_y);
            mavgVz.update(target_z - current_z);
            go_close_velocity.header.frame_id = "uav_1_home";
            go_close_velocity.twist.linear.x = mavgVx.average;
            go_close_velocity.twist.linear.y = mavgVy.average;
            go_close_velocity.twist.linear.z = mavgVz.average;
            go_close_velocity.twist.angular.x = 0.0;
            go_close_velocity.twist.angular.y = 0.0;
            go_close_velocity.twist.angular.z = 0.0;
            // set_velocity.request.velocity = go_close_velocity;
            // srvSetVelocity.call(set_velocity);
            pub_set_velocity.publish(go_close_velocity);
            // ual.setVelocity(go_close_velocity);
            fileMovingAvg << mavgVx.average << " "
                          << mavgVy.average << " "
                          << mavgVz.average << "\n";
            // Escribe en un .dat
            fileVelocity << go_close_velocity.twist.linear.x << " " << current_velocity_x << " "
                         << go_close_velocity.twist.linear.y << " " << current_velocity_y << " "
                         << go_close_velocity.twist.linear.z << " " << current_velocity_z << "\n";
            filePosition << target_x << " " << current_x << " "
                         << target_y << " " << current_y << " "
                         << target_z << " " << current_z << "\n";
            fileActualLA << look_ahead << " " << new_vectorT[p] << "\n";
            // Publica
            pub_normal_distance.publish(path_normal_distance);
            pub_to_target.publish(path_to_target_distance);
            pub_look_ahead.publish(path_look_ahead);
            pub_visualization_marker.publish(marker);
            currentTrajectory();
            pub_draw_current_path.publish(msg_current_draw);
            i++;
            // Espera
            ros::Duration(0.05).sleep();
            ros::spinOnce();
            if (!(normal_distance < look_ahead && to_target_distance > look_ahead)) {
                continue;
            }
        }
        // El dron está fuera del path. Se le otorga la velocidad necesaria para que vaya hacia
        // el waypoint objetivo.
        while (normal_distance > look_ahead) {
            normal_distance = calculateNormalDistance();
            mavgVx.update(target_x - current_x);
            mavgVy.update(target_y - current_y);
            mavgVz.update(target_z - current_z);
            go_close_velocity.header.frame_id = "uav_1_home";
            go_close_velocity.twist.linear.x = mavgVx.average;
            go_close_velocity.twist.linear.y = mavgVy.average;
            go_close_velocity.twist.linear.z = mavgVz.average;
            go_close_velocity.twist.angular.x = 0.0;
            go_close_velocity.twist.angular.y = 0.0;
            go_close_velocity.twist.angular.z = 0.0;
            // set_velocity.request.velocity = go_close_velocity;
            // srvSetVelocity.call(set_velocity);
            pub_set_velocity.publish(go_close_velocity);
            // ual.setVelocity(go_close_velocity);
            fileMovingAvg << mavgVx.average << " "
                          << mavgVy.average << " "
                          << mavgVz.average << "\n";
            // Escribe en un .dat
            fileVelocity << go_close_velocity.twist.linear.x << " " << current_velocity_x << " "
                         << go_close_velocity.twist.linear.y << " " << current_velocity_y << " "
                         << go_close_velocity.twist.linear.z << " " << current_velocity_z << "\n";
            filePosition << target_x << " " << current_x << " "
                         << target_y << " " << current_y << " "
                         << target_z << " " << current_z << "\n";
            fileActualLA << look_ahead << " " << new_vectorT[p] << "\n";
            // Publica
            currentTrajectory();
            pub_draw_current_path.publish(msg_current_draw);
            pub_visualization_marker.publish(marker);
            i++;
            // Espera
            ros::Duration(0.05).sleep();
            ros::spinOnce();
            if (!(normal_distance > look_ahead)) {
                continue;
            }
        }
        // Calcula la suma de todas las distancias normales.
        calculateSumNormalDistance();
        // Cuenta los cambios de waypoint.
        cont++;
    }
    // Escribe cuanto tiempo se ha tardado en hacer la mision en un .dat
    fileT << ros::Time::now() - begin << std::endl;
    // Cierra el .dat
    fileVelocity.close();
    filePosition.close();
    fileMovingAvg.close();
    fileActualLA.close();
    fileT.close();
    // ---------------------------------------------- MISION ----------------------------------------------
    std::cout << "[ TEST] Mission complete! " << std::endl;
    std::cout << "[ TEST] Time: " << ros::Time::now() - begin << std::endl;
    std::cout << "[ TEST] Media normal_distance: " << calculateSumNormalDistance() / cont << '\n';

    // ual.land(1);
    land.request.blocking = true;
    srvLand.call(land);
}