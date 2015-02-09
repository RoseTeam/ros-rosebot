#include "rosebot.h"
#include "robomovies.h"
#include <ros/ros.h>
#include <cmath>


Rosebot::Rosebot(Vec pos0, Vec orient0, Vec* but)
{

	arene_ = createRobomoviesMap();

	_acc_w_ = _acc_v_ / voie * 2.; // rad/s/s
	_acc_wf_ = _acc_f_ / voie * 2.; // rad/s/s
	_acc_cen_ = 550; // m/s/s acceleration tangentielle max avant decrochage


	but_  = but;

	pos_	= pos0;
	orient_ = orient0;

	vertex_ = arene_->insertVertex_py(pos_,std::vector<double>(1,rayon));
	assert(vertex_->coords == pos_);
	ROS_INFO("Rosebot Creation %.2f,%.2f", pos_[0], pos_[1]);
}


void Rosebot::PreBouge(double deltaT)
{

	//obj_v = obj_w = 0

	double rpivot, rpivot_suiv;
	Vec* but_suiv;
	itineraire.maj_pos(rayon, but_, rpivot, but_suiv, rpivot_suiv);
	bool but_decellerer = true;

	ROS_INFO("Nouvel iti %.2f,%.2f", (*but_)[0], (*but_)[1]);


	double vdirecte = orient_ * _v_;


	if (!but_)
	{
		double dv_max = deltaT * _acc_f_;

		// En l'absence d'objectif, orientation dans la direction du deplacement
		// pas d'objectif :  s'arreter au plus vite.

		if( vdirecte > 0)
		{
			_eff_ = -std::min<double>(dv_max, vdirecte) * orient_;
		}
		else if( vdirecte < 0 )
			_eff_ = std::min<double>(dv_max, -vdirecte) * orient_;

		_w_ = 0;
	}
	else
	{
		//ROS_INFO("Nouvel iti %.2f,%.2f", (*but_)[0], (*but_)[1]);

		//ROS_INFO("but present");
		double but_tol = rayon/10;

		Vec versBut = *but_ - pos_;
		Vec versSuiv = versBut;

		bool pre_pivot = false;

		if(rpivot)
		{
			versSuiv = *but_suiv - pos_;
			Vec axeBut = *but_suiv - *but_;

			if( axeBut * versBut <= 0 && rpivot * (axeBut ^ versBut) <= 0 )
			{
				itineraire.increme();
				versSuiv += rpivot_suiv*orient_.nml();
				versBut = versSuiv;
				but_ = but_suiv;
				rpivot = 0;
			}
			else
			{
				if( ! versBut.tan_abs_min(-rpivot*orient_.nml(), .1) )
				{
					// va vers prox sans encore tourner
					pre_pivot = true;
					Vec marge = std::min<double>(fabs(rpivot),versBut.nor()) * orient_.nml();
					if( rpivot > 0 )
						versBut += marge;
					else
						versBut -= marge;
				}
				else if( rpivot_suiv )
				{
					Vec marge = std::min<double>(fabs(rpivot_suiv),versSuiv.nor()) * orient_.nml();
					if( rpivot > 0 )
						versSuiv += marge;
					else
						versSuiv -= marge;
				}
			}
		}

		if(true)
		{
			double dv_max = _acc_f_ * deltaT;
			double vn = orient_ ^  _v_;

			if( vdirecte < -_acc_f_ * .1 )
			{
				// va a reculons : freine vers l'avant
				// print 'freine reculons', vdirecte;
				_eff_ = std::min<double>(dv_max, -vdirecte) * orient_;
				Sorienter(deltaT, versBut);
			}
			else if( ! versBut.tan_abs_min(orient_,0.1) || (rpivot != 0 && ! pre_pivot) )
			{

				// vitesse et orient alignees
				// mais versBut et orient non alignees
				if (pre_pivot || rpivot == 0){// && ! versBut.tan_abs_min(orient_,1):
					// rotation seule

					if( vdirecte ){ //annule une vitesse residuelle
						if( vdirecte > 0 )
							_eff_ = -std::min<double>(dv_max, vdirecte) * orient_;
						else
							_eff_ = std::min<double>(dv_max, -vdirecte) * orient_;
					}
					if(versBut > but_tol || pre_pivot){
						// print "reorient freine", orient_, 'versBut', versBut, 'vx %.2f'% vdirecte
						Sorienter(deltaT, versBut);
					}
					else
					{
						// print 'iti increme 1'
						itineraire.increme();
						but_ = nullptr;
					}
				}
				else if( rpivot_suiv != 0 || versSuiv > 2 * rayon )
				{

					// print 'pivoter', pre_pivot
					pivoter((pre_pivot ? versBut : versSuiv), deltaT, (pre_pivot ? 0 : rpivot), *but_);
				}
				else
				{
					if(versBut > but_tol)
					{
						// reorientation seule
						// print "reorient seule", versBut
						Sorienter(deltaT, versBut);
					}
					else{
						// print 'iti increme 2'
						itineraire.increme();
						but_ = nullptr;
					}
				}
			}
			else
			{
				// aligne sur le but, pas de virage a prendre

				Sorienter(deltaT, versBut);

				// print "aligne sur le but", versBut, orient_ * 1000

				// self.vdir = versBut.uni()

				Vec versButUni = orient_;
				double vdirecte = versButUni * _v_;

				double obj_v = 0; // but terminal

				if( pre_pivot || rpivot != 0 )
				{
					// virage a prendre devant
					//versPivot = but_-pos_
					obj_v = sqrt(fabs(rpivot)*_acc_cen_);
				}

				/*
				_dpos, obstacles = self.VerifObstacles(versBut)

				for(auto obst : obstacles )
				{
					if isinstance(obst, tryangle.Wertex):
						if obst.rayon == 0 && (but_-obst.pos)*orient_ > 0:
							// obstacle imprevu pris en compte si il est plus proche que le prochain point d'itineraire
							cote = 1 if (obst.pos-pos_)^(but_-pos_) > 0 else -1
							itineraire.ajout(obst.pos, cote=cote*2*rayon*self.surrayon, enTete=True)
							break
					else:
						#obstacle segment imprevu
						if orient_ * obst.univec < 0:
							som = obst.org
							cote = 1
						else:
							som = obst.dest
							cote = -1

						if (but_-som.coords) * orient_ > 0:
							itineraire.ajout(som.coords, cote=cote*2*rayon*self.surrayon, enTete=True)
							break
				}
				 */
				double acc_f_usu = _acc_f_;
				double dv_frein_max = deltaT * _acc_f_; // vitesse annulable en un seul tour en freinage max
				double dv_frein_usu = deltaT * acc_f_usu;    // reduction vitesse en freinage usuel (non max)

				double d_freinage = 0;
				bool decelProche = false;

				// Distance de freinage (d tel que: v - a*t = 0 et v-1/2*a^2*t = d)
				// d = 1/2 * v^2 / a
				if(vdirecte > obj_v)
				{
					// distance theorique (temps continu) pour annuler la vitesse avec un freinage maximum et constant.
					// en premiere approx, suppose le freinage visqueux constant
					d_freinage  = (vdirecte*vdirecte-obj_v*obj_v*2) * .5 / acc_f_usu;
					decelProche = versBut <= d_freinage + but_tol;
				}

				if(  versBut <= but_tol && ( ! but_decellerer || _v_ < dv_frein_max ) )
				{
					// on est proche du but_
					// print 'proche du but', versBut, '_v_', _v_, 'dv max', dv_frein_max
					if( but_decellerer)
						_eff_ -= _v_; // annulons la vitesse

					itineraire.increme();
				}
				else
				{

					bool doit_freiner = false; // si l'on va dans le sens oppose au but, il faut inverser en commencant par freiner

					double obj_v = 0;
					if( vdirecte > 0)
					{
						if( decelProche && ! doit_freiner && but_decellerer && vdirecte != obj_v )
						{
							// nombre de tours a freinage constant pour annuler la vitesse (suppose deltaT constant)
							double n_frein = (int)(.5 + (2*versButUni*versBut/(vdirecte-obj_v)/deltaT+1));
							dv_frein_usu = std::min<double>(dv_frein_max, vdirecte / n_frein);
							// distance de freinage en supposant un freinage constant pendant n-1 tours:
							// dd_freinage = Sigma( v - dv * i )[i=1 a n_frein-1] * delta T
							// dd_freinage = (n_frein-1) * ( abs(vdirecte) - dv_frein_usu*n_frein/2. ) * deltaT
							doit_freiner  = true;
						}
					}
					if( doit_freiner )
					{
						// Phase de freinage si l'on est proche du but ou que l'on va dans le mauvais sens.
						// Calcul du temps d'arret en supposant un freinage maximum et constant.
						// (et supposant deltaT constant aussi pour annuler la vitesse).
						double v_reduc = -std::min<double>(dv_frein_usu, vdirecte-obj_v);

						_eff_ = orient_ * v_reduc;
						// print 'freinage', 'd_frei', d_freinage, 'versBut', versBut
					}
					else
					{
						// Phase d'acceleration
						// print 'phase accel, vdir', vdirecte, orient_*100


						dv_max = _acc_v_ * deltaT;
						_eff_ = orient_ * dv_max;
					}
				}
			}
		}
	}
	_v_ += _eff_;
}

void Rosebot::Sorienter(double deltaT, Vec direction, double obj_w)
{
	//""" True si l'agent est aligne sur la direction souhaitee"""

	double angle = std::atan2(orient_^direction, orient_*direction);

	double angle_tol = 1e-2;

	if( fabs(angle) <= angle_tol )
	{
		_w_ = 0;;
	}
	else
	{
		double a_d = (_w_ *_w_* 2 - obj_w*obj_w*2) / 2. / _acc_wf_;

		if( abs(angle) < 1.05*a_d )
		{
			//# freinage
			//#print 'freine angle', angle, 'a_d', a_d
			double acc_a = std::min<double>( _acc_wf_, (_w_ *_w_* 2 - obj_w*obj_w*2) / 2. / fabs(angle) );
			if(angle < 0)
			{
				_w_ += acc_a * deltaT;
				_w_ = std::min<double>(_w_, 0);
			}
			else
			{
				_w_ -= acc_a * deltaT;
				_w_ = std::max<double>(_w_, 0);
			}

		}
		else if(abs(angle) > 1.1*a_d)
		{
			//#accel
			//#print 'accel angle', angle, 'a_d', a_d

			if( angle < 0 )
				_w_ -= _acc_w_ * deltaT;
			else
				_w_ += _acc_w_ * deltaT;
		}
		else
		{
			//# ni accel ni freinage
			//#print 'nini angle', angle, 'a_d', a_d

		}
	}
}

