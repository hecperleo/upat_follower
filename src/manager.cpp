#include <pathFollower/manager.h>

Manager::Manager()
{
    n = ros::NodeHandle();
    params();
    // Subscriptions
    subPose = n.subscribe("/uav_1/ual/pose", 0, &Manager::UALPoseCallback, this);
	subPath = n.subscribe("initPath", 0, &Manager::UALPathCallback, this);
	subvectorT = n.subscribe("vectorT", 0, &Manager::vectorTCallback, this);

    // Publishers
    pubDrawPath = n.advertise<nav_msgs::Path>("drawInitPath", 1000);
	pubEclPath = n.advertise<nav_msgs::Path>("posSpline", 1000);
	pubEclPathV = n.advertise<nav_msgs::Path>("velSpline", 1000);
	pubEclPath1 = n.advertise<nav_msgs::Path>("posSpline1", 1000);
	pubEclPath2 = n.advertise<nav_msgs::Path>("posSpline2", 1000);
	pubnewvectorT = n.advertise<nav_msgs::Path>("newVectorT", 1000);

    loop();
}

Manager::~Manager()
{
}

void Manager::params()
{
    // ros::NodeHandle nparam("~");
    // if (nparam.getParam("phase", phase))
    // {
    //     ROS_WARN("Got Manager param phase: %i", phase);
    // }
    // else
    // {
    //     ROS_WARN("Failed to get Manager param phase: %i", phase);
    // }
}

void Manager::funcPreprocesamiento()
{
	float dist_entreWp, tempo, tempoMin, vel;
	float sumTempo = 0;
	float sumTempoMin = 0;
	for (int i = 0; i < poseListX.size() - 1; i++)
	{
		dist_entreWp = funcMod(poseListX[i], poseListX[i + 1],
							   poseListY[i], poseListY[i + 1],
							   poseListZ[i], poseListZ[i + 1]);
		vel = velocidadMax / vectorT[i];
		tempo = dist_entreWp / vel;
		tempoMin = dist_entreWp / velocidadMax;
		//std::cout << "[ ... ] Tempo    = " << tempo << '\n';
		//std::cout << "[ ... ] TempoMin = " << tempoMin << '\n';
		sumTempo = sumTempo + tempo;
		sumTempoMin = sumTempoMin + tempoMin;
	}
	//std::cout << "[ ... ] sumTempo        = " << sumTempo << '\n';
	//std::cout << "[ ... ] sumTempoMin     = " << sumTempoMin << '\n';
}

void Manager::funcCompruebaTiempos()
{
	for (int i = 0; i < vectorT.size(); i++)
	{
		if (vectorT[i] < 1)
		{
			vectorT[i] = 1;
		}
		if (vectorT[i] > 3.5)
		{
			vectorT[i] = 3.5;
		}
		//std::cout << "[ ... ] vectorT[" << i << "] = " << vectorT[i] << '\n';
	}
}

/* CALLBACKS */
void Manager::UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg1)
{
	// Corrección respecto del mapa
	actualPosX = msg1->pose.position.x;
	actualPosY = msg1->pose.position.y;
	actualPosZ = msg1->pose.position.z;

	return;
}

void Manager::UALPathCallback(const nav_msgs::Path &msg)
{
	double poseX, poseY, poseZ;
	std::vector<grvc::ual::Waypoint> poseList;
	// Corrección respecto del mapa
	msgDrawPath.header.frame_id = "map";
	msgDrawPath.poses = msg.poses;
	if (flagSubPath == true)
	{
		for (auto p : msg.poses)
		{
			waypoint.pose.position.x = p.pose.position.x;
			waypoint.pose.position.y = p.pose.position.y;
			waypoint.pose.position.z = p.pose.position.z;
			poseList.push_back(waypoint);
			poseX = p.pose.position.x;
			poseListX.push_back(poseX);
			poseY = p.pose.position.y;
			poseListY.push_back(poseY);
			poseZ = p.pose.position.z;
			poseListZ.push_back(poseZ);
		}
		for (int q = 0; q < poseList.size(); q++)
		{
			waypoint.pose.position.x = poseList[q].pose.position.x;
			waypoint.pose.position.y = poseList[q].pose.position.y;
			waypoint.pose.position.z = poseList[q].pose.position.z;
			path.poses.push_back(waypoint);
		}
		std::cout << "[ TEST] Running!" << '\n';
		std::cout << "[ TEST] Path size = " << path.poses.size() << '\n';
	}
	flagSubPath = false;

	return;
}

void Manager::vectorTCallback(const nav_msgs::Path &msg)
{
	if (flagSubvectorT == true)
	{
		std::ofstream fileVectorT;
		fileVectorT.open("/home/hector/Matlab_ws/vectorT.dat");
		for (int p = 0; p < msg.poses.size(); p++)
		{
			vectorT.push_back(msg.poses.at(p).pose.position.x);
			fileVectorT << msg.poses.at(p).pose.position.x << "\n";
		}
		fileVectorT.close();
	}
	flagSubvectorT = false;
	return;
}
/* CALLBACKS */

nav_msgs::Path Manager::Construct_Path_Msg(double *x, double *y, double *z, int length)
{
	nav_msgs::Path msg;
	std::vector<geometry_msgs::PoseStamped> poses(length);
	msg.header.frame_id = "map";
	for (int i = 0; i < length; i++)
	{
		poses.at(i).pose.position.x = x[i];
		poses.at(i).pose.position.y = y[i];
		poses.at(i).pose.position.z = z[i];
	}
	msg.poses = poses;
	return msg;
}

float Manager::funcMod(float x1, float x2, float y1, float y2, float z1, float z2)
{
	float mod;
	mod = sqrt((x2 - x1) * (x2 - x1) +
			   (y2 - y1) * (y2 - y1) +
			   (z2 - z1) * (z2 - z1));
	return mod;
}

void Manager::eclSpline(float minT, float dist_total)
{
	if (lastOne == false)
	{					// Comentar para tener el generador V1
		t = dist_total; // Comentar para tener el generador V1
						//t = 0; // Descomentar para tener el generador V1
	}					// Comentar para tener el generador V1
	if (lastOne == true)
	{
		// std::cout << "[ TEST] lastOne " << lastOne << " | flagFinishSpline " << flagFinishSpline << " | t " << t << '\n';
	}
	bool safe = false;
	while (safe == false)
	{
		newPoseListX = interpolaWps(poseListX, t);
		newPoseListY = interpolaWps(poseListY, t);
		newPoseListZ = interpolaWps(poseListZ, t);

		int wp_total = newPoseListX.size();
		double *wp_x = (double *)malloc(sizeof(double) * wp_total);
		double *wp_y = (double *)malloc(sizeof(double) * wp_total);
		double *wp_z = (double *)malloc(sizeof(double) * wp_total);
		std::ofstream fileWP, fileSpline;
		fileWP.open("/home/hector/Matlab_ws/wp.dat");
		fileSpline.open("/home/hector/Matlab_ws/spline.dat");
		for (int i = 0; i < wp_total; i++)
		{
			wp_x[i] = newPoseListX[i];
			wp_y[i] = newPoseListY[i];
			wp_z[i] = newPoseListZ[i];
			fileWP << newPoseListX[i] << " " << newPoseListY[i] << " " << newPoseListZ[i] << " "
				   << newPoseListX[i] << " " << newPoseListY[i] << " " << newPoseListZ[i] << " "
				   << newPoseListX[i] << " " << newPoseListY[i] << " " << newPoseListZ[i] << "\n";
		}
		fileWP.close();
		//spline fitting
		double *x_ptr;
		double *y_ptr;
		double *z_ptr;
		x_ptr = wp_x;
		y_ptr = wp_y;
		z_ptr = wp_z;
		//nav_msgs::Path spline_msg;
		ecl::CubicSpline spline_x, spline_y, spline_z;
		ecl::TensionSpline spline_x2, spline_y2, spline_z2;
		//find a spline to fit the linear path
		ecl::Array<double> t_set(wp_total), x_set(wp_total), y_set(wp_total), z_set(wp_total);
		for (int i = 0; i < wp_total; i++)
		{
			x_set[i] = x_ptr[i];
			y_set[i] = y_ptr[i];
			z_set[i] = z_ptr[i];
		}
		if (lastOne == false)
		{
			for (int i = 0; i < wp_total; i++)
			{
				t_set[i] = (double)i;
			}
		}
		if (lastOne == true)
		{
			for (int i = 0; i < wp_total; i++)
			{
				t_set[i] = (double)lastSplinevectorT[i];
			}
		}
		//spline fit with ECL geometry

		spline_x = ecl::CubicSpline::Natural(t_set, x_set);
		spline_y = ecl::CubicSpline::Natural(t_set, y_set);
		spline_z = ecl::CubicSpline::Natural(t_set, z_set);
		//using ecl::SmoothLinearSpline;
		double max_curvature = 20.0; // Por debajo de este valor, no funciona.
		double tension = 10.0;

		/*ecl::SmoothLinearSpline spline_x1(t_set, x_set, max_curvature);
		ecl::SmoothLinearSpline spline_y1(t_set, y_set, max_curvature);
		ecl::SmoothLinearSpline spline_z1(t_set, z_set, max_curvature);*/
		/*spline_x2 = ecl::TensionSpline::Natural(t_set, x_set, tension);
		spline_y2 = ecl::TensionSpline::Natural(t_set, y_set, tension);
		spline_z2 = ecl::TensionSpline::Natural(t_set, z_set, tension);*/

		int spline_pts;
		if (lastOne == false)
		{
			spline_pts = (wp_total - 1) * 100;
		}
		if (lastOne == true)
		{
			spline_pts = (lastSplinevectorT.back()) * 100;
		}
		double sx[spline_pts], svx[spline_pts], sax[spline_pts];
		double sx1[spline_pts], svx1[spline_pts], sax1[spline_pts];
		double sx2[spline_pts], svx2[spline_pts], sax2[spline_pts];
		double sy[spline_pts], svy[spline_pts], say[spline_pts];
		double sy1[spline_pts], svy1[spline_pts], say1[spline_pts];
		double sy2[spline_pts], svy2[spline_pts], say2[spline_pts];
		double sz[spline_pts], svz[spline_pts], saz[spline_pts];
		double sz1[spline_pts], svz1[spline_pts], saz1[spline_pts];
		double sz2[spline_pts], svz2[spline_pts], saz2[spline_pts];
		std::vector<double> vectorVz;
		if (lastOne == true)
		{
			ROS_WARN("-1-");
		}

		for (int i = 0; i < spline_pts; i++)
		{
			sx[i] = spline_x(i / 100.0);
			sy[i] = spline_y(i / 100.0);
			sz[i] = spline_z(i / 100.0);
			svx[i] = spline_x.derivative(i / 100.0);
			svy[i] = spline_y.derivative(i / 100.0);
			svz[i] = spline_z.derivative(i / 100.0);
			sax[i] = spline_x.dderivative(i / 100.0);
			say[i] = spline_y.dderivative(i / 100.0);
			saz[i] = spline_z.dderivative(i / 100.0);
			/*sx1[i] 	= spline_x1(i/100.0);
			sy1[i] 	= spline_y1(i/100.0);
			sz1[i] 	= spline_z1(i/100.0);
			svx1[i] = spline_x1.derivative(i/100.0);
			svy1[i] = spline_y1.derivative(i/100.0);
			svz1[i] = spline_z1.derivative(i/100.0);
			sax1[i] = spline_x1.dderivative(i/100.0);
			say1[i] = spline_y1.dderivative(i/100.0);
			saz1[i] = spline_z1.dderivative(i/100.0);*/
			/*sx2[i] 	= spline_x2(i/100.0);
			sy2[i] 	= spline_y2(i/100.0);
			sz2[i] 	= spline_z2(i/100.0);
			svx2[i] = spline_x2.derivative(i/100.0);
			svy2[i] = spline_y2.derivative(i/100.0);
			svz2[i] = spline_z2.derivative(i/100.0);
			sax2[i] = spline_x2.dderivative(i/100.0);
			say2[i] = spline_y2.dderivative(i/100.0);
			saz2[i] = spline_z2.dderivative(i/100.0);*/
			vectorVz.push_back(spline_z.derivative(i / 100.0));
			if (lastOne == true)
			{
				ROS_WARN("-2-");
			}
		}
		if (lastOne == true)
		{
			ROS_WARN("-3-");
		}
		spline_msg = Construct_Path_Msg(sx, sy, sz, sizeof(sx) / sizeof(double));
		//spline_msg1 = Construct_Path_Msg(sx1, sy1, sz1, sizeof(sx1)/sizeof(double));
		//spline_msg2 = Construct_Path_Msg(sx2, sy2, sz2, sizeof(sx2)/sizeof(double));
		splineV_msg = Construct_Path_Msg(svx, svy, svz, sizeof(svx) / sizeof(double));
		for (int i = 0; i < spline_msg.poses.size(); i++)
		{
			fileSpline << sx[i] << " " << sy[i] << " " << sz[i] << " "
					   //<< sx1[i] 		<< " " << sy1[i] 		<< " " << sz1[i] 		<< " "
					   //<< sx2[i] 		<< " " << sy2[i] 		<< " " << sz2[i] 		<< " "
					   << svx[i] << " " << svy[i] << " " << svz[i] << " "
					   //<< svx1[i] 	<< " " << svy1[i] 	<< " " << svz1[i] 	<< " "
					   //<< svx2[i] 	<< " " << svy2[i] 	<< " " << svz2[i] 	<< " "
					   << sax[i] << " " << say[i] << " " << saz[i] //<< " "
																   //<< sax1[i]		<< " " << say1[i]		<< " " << saz1[i]		<< " "
																   /* << sax2[i]		<< " " << say2[i]		<< " " << saz2[i]	 */
					   << "\n";
		}
		//std::cout << "path ok size = " << spline_msg.poses.size() << '\n';
		fileSpline.close();
		//safe = true;
		// Comentar para tener el generador V1
		// Calculo de la menor Vz (variable más restrictiva)
		auto smallest_Vz_min = std::min_element(vectorVz.begin(), vectorVz.end());
		int tiempo = t;

		if (lastOne == true)
		{
			std::cout << "[ TEST] Ultimo calculo de spline" << '\n';
			if (*smallest_Vz_min < -1)
			{
				t = t + 1;
				lastOne = false;
				safe = false;
			}
			else if (*smallest_Vz_min > -1)
			{
				funcInterpvectorT(spline_pts);
				safe = true;
				flagFinishSpline = true;
				std::cout << "[ TEST] Terminado" << '\n';
				std::cout << "[ TEST] Spline size     = " << spline_pts << '\n';
				std::cout << "[ TEST] newVectorT size = " << newvectorT.size() << '\n';
				std::cout << "[ TEST] Vz min          = " << *smallest_Vz_min << '\n';
				std::cout << "[ TEST] Distancia total = " << dist_total << '\n';
				std::cout << "[ TEST] Tiempo minimo   = " << t << '\n';
				std::cout << "[ TEST] Tiempo aprox    = " << sumVecT * t / vectorT.size() << '\n';
			}
		}
		else if (lastOne == false && safe == false)
		{
			safe = funcOtraSpline(vectorVz, spline_pts, safe);
			if (safe == true)
			{
				std::cout << "[ TEST] Calcula otra spline" << '\n';
				std::cout << "[ TEST] Calcula la ultima" << '\n';
				lastOne = true;
				funcInterpvectorT(spline_pts);
				flagFinishSpline = true;
			}
		}
		// std::cout << "[ TEST] safe " << safe << " | lastOne " << lastOne << " | flagFinishSpline " << flagFinishSpline << '\n';
		// Comentar para tener el generador V1
	}
}

bool Manager::funcOtraSpline(std::vector<double> vVz, int splineSize, bool safe)
{
	auto smallest_Vz_min = std::min_element(vVz.begin(), vVz.end());
	int tiempo = t;
	if ((tiempo % vectorT.size()) != 0 || (splineSize % vectorT.size()) != 0)
	{
		t = t + 1;
	}
	else if ((tiempo % vectorT.size()) == 0 && (splineSize % vectorT.size()) == 0)
	{
		safe = true;
		lastSplinevectorT = funcAumentaVector(vectorT, tiempo);
	}
	return safe;
}

void Manager::funcInterpvectorT(int splineSize)
{
	std::ofstream fileNewVectorT;
	fileNewVectorT.open("/home/hector/Matlab_ws/newVectorT.dat");
	for (int i = 0; i < vectorT.size(); i++)
	{
		for (int j = 0; j < (splineSize / vectorT.size()); j++)
		{
			newvectorT.push_back(vectorT[i]);
			fileNewVectorT << vectorT[i] << "\n";
		}
	}
	fileNewVectorT.close();

	std::vector<grvc::ual::Waypoint> newTList;
	grvc::ual::Waypoint newT;
	for (int p = 0; p < newvectorT.size(); p++)
	{
		newT.pose.position.x = newvectorT[p];
		newTList.push_back(newT);
	}
	std::vector<geometry_msgs::PoseStamped> times(newTList.size());
	for (int p = 0; p < newTList.size(); p++)
	{
		times.at(p).pose.position.x = newTList[p].pose.position.x;
	}
	msgnewvectorT.poses = times;
}

std::vector<double> Manager::funcAumentaVector(std::vector<double> vect, int finalSize)
{
	std::vector<double> newVector;
	std::vector<double> otroVector;
	int t = 0;
	newVector.push_back(t);
	for (int i = 0; i < vect.size(); i++)
	{
		for (int j = 0; j < (finalSize / vect.size()); j++)
		{
			t = t + vect[i];
			newVector.push_back(t);
			//std::cout << "[ TEST] VecValue: " << vect[i] << " | t: " << t << '\n';
		}
	}

	for (int i = 0; i < newVector.size() - 1; i++)
	{
		otroVector.push_back(newVector[i]);
	}

	return otroVector;
}

std::vector<double> Manager::interpolaWps(std::vector<double> wp, double t)
{
	std::vector<double> t_axis;
	std::vector<double> newWpList;
	double porcion;
	double newTsize = t;
	if (newTsize < wp.size())
	{
		newTsize = wp.size();
	}
	for (int i = 1; i <= wp.size(); i++)
	{
		t_axis.push_back(i);
	}
	double newWp = t_axis.front();
	newWpList.push_back(newWp);
	porcion = (t_axis.back() - t_axis.front()) / (newTsize - 1.0);
	for (int i = 1; i < newTsize; i++)
	{
		newWp = newWp + porcion;
		newWpList.push_back(newWp);
	}
	auto res = interp1(t_axis, wp, newWpList);

	return res;
}

template <typename Real> int Manager::nearestNeighbourIndex(std::vector<Real> &x, Real &value)
{
	Real dist = std::numeric_limits<Real>::max();
	Real newDist = dist;
	size_t idx = 0;

	for (size_t i = 0; i < x.size(); ++i)
	{
		newDist = std::abs(value - x[i]);
		if (newDist <= dist)
		{
			dist = newDist;
			idx = i;
		}
	}

	return idx;
}

template <typename Real> std::vector<Real> Manager::interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new)
{
	std::vector<Real> y_new;
	Real dx, dy, m, b;
	size_t x_max_idx = x.size() - 1;
	size_t x_new_size = x_new.size();

	y_new.reserve(x_new_size);

	for (size_t i = 0; i < x_new_size; ++i)
	{
		size_t idx = nearestNeighbourIndex(x, x_new[i]);

		if (x[idx] > x_new[i])
		{
			dx = idx > 0 ? (x[idx] - x[idx - 1]) : (x[idx + 1] - x[idx]);
			dy = idx > 0 ? (y[idx] - y[idx - 1]) : (y[idx + 1] - y[idx]);
		}
		else
		{
			dx = idx < x_max_idx ? (x[idx + 1] - x[idx]) : (x[idx] - x[idx - 1]);
			dy = idx < x_max_idx ? (y[idx + 1] - y[idx]) : (y[idx] - y[idx - 1]);
		}

		m = dy / dx;
		b = y[idx] - x[idx] * m;

		y_new.push_back(x_new[i] * m + b);
	}

	return y_new;
}

void Manager::loop()
{
    while (ros::ok())
    {
        if (path.poses.size() != 0 && flagSpline == true)
		{
			for (int i = 0; i < vectorT.size(); i++)
			{
				sumVecT = sumVecT + vectorT[i];
			}
			std::cout << "[ TEST] sumVecT = " << sumVecT << '\n';
			float distTotal = 0;
			for (int i = 0; i < poseListX.size() - 1; i++)
			{
				distTotal = distTotal + funcMod(poseListX[i], poseListX[i + 1],
												poseListY[i], poseListY[i + 1],
												poseListZ[i], poseListZ[i + 1]);
			}
			funcCompruebaTiempos();
			while (flagFinishSpline == false)
			{ // Comentar para tener el generador V1
				eclSpline(sumVecT, distTotal);
			} // Comentar para tener el generador V1
			funcPreprocesamiento();
			//std::cout << "[ TEST] distTotal = " << distTotal << '\n';
			flagSpline = false;
		}
		// Publicacion de splines
		pubEclPath.publish(spline_msg);
		pubEclPathV.publish(splineV_msg);
		pubEclPath1.publish(spline_msg1);
		pubEclPath2.publish(spline_msg2);
		// Publicacion de ambas trayectorias
		pubDrawPath.publish(msgDrawPath);
		pubnewvectorT.publish(msgnewvectorT);

		i++;
		sleep(0.1);
		ros::spinOnce();
    }
}