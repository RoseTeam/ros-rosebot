/*
 * rosebot.h
 *
 *  Created on: Feb 7, 2015
 *      Author: gg99
 */

#ifndef SRC_ROSEBOT_H_
#define SRC_ROSEBOT_H_

#include "point2d.h"
#include "itineraire.h"
#include "mesh.h"

typedef Point2d Vec;

class Rosebot : public wertex
{
public:

	Rosebot(Vec pos0 = Vec(1000,350), Vec orient0 = Vec(1,0), Vec* but=nullptr);

	shared_ptr<Mesh> arene_;

	wertex* vertex_;

    double _acc_v_     = 300; // mm/s acceleration ligne droite
    double _acc_f_     = _acc_v_; // mm/s decelleration de freinage
    double _acc_w_ = _acc_v_ / voie * 2.; // rad/s/s
    double _acc_wf_ = _acc_f_ / voie * 2.; // rad/s/s
    double _acc_cen_ = 550; // m/s/s acceleration tangentielle max avant decrochage
    double _v_rad = 1.5; //rad/s

    double rayon = 150;
    double voie = 230.;

    double v_max = 800; //mm/s
    double w_max = 1.; //rad/s

    Vec pos_;
    Vec orient_;
    Vec* but_= nullptr;
    Vec _v_;
    Vec _eff_;

	double _w_ = 0;

	Itineraire itineraire;

	void Sorienter(double deltaT, Vec direction, double obj_w=0);


	void pivoter(Vec versBut, double deltaT, double rpivot, Vec pivot)
	{
        double vnor = _v_ * orient_;
        double obj_v;
        Vec versPivot;

        if( rpivot )
		{
            versPivot = pivot-pos_;
            obj_v = sqrt(fabs(versPivot.nor())*_acc_cen_);
		}
        else
        {
            // tourne au plus serre
            if(versBut * orient_ <= 0)
                obj_v = 0;
            else
                obj_v = sqrt(2*rayon*_acc_cen_);
        }

        Vec nov_v = orient_.copy();

        if( vnor > obj_v)// && versBut * orient_ < 0 )
		{    // decellere
            vnor = std::max<double>(vnor - _acc_f_*deltaT, obj_v);
            if( vnor == 0 )
            {
                _eff_ = - _v_;
                Sorienter(deltaT, versBut);
                return;
            }
		}
        else if( vnor < obj_v)
        {
            // accelere
            vnor = std::min<double>(vnor + _acc_v_*deltaT, obj_v);
        }
        else if( vnor == 0 )
		{
            Sorienter(deltaT, versBut);
            return;
		}

        double ww = std::min<double>(_v_rad, _acc_cen_ / vnor);

        if( (rpivot && rpivot > 0) || (orient_ ^ (pivot-pos_) < 0 ))
            ww *= -1; // sens de rotation

        double deltaW = ww*deltaT;

        nov_v.irot(deltaW);

        if ( ((nov_v ^ versBut >= 0) ^ (deltaW > 0)) && nov_v * versBut > 0)
        {
            nov_v = versBut.uni();

            if( rpivot && ((orient_ ^ versBut < 0) ^ (rpivot < 0)))
            {
            	itineraire.increme();
            	but_= nullptr;
            }
        }
        nov_v *= vnor;

        _eff_ = nov_v - _v_;

        orient_ = nov_v.uni();

	};

	void AssBut(Vec* but=nullptr, Vec* orient=nullptr)
	{
		if( !but )
		{
			itineraire = Itineraire();
		}
		else
		{
			//Algo = PathFindAlgo.MinDist if mods & pygame.KMOD_ALT else PathFindAlgo.MinDistToEnd

			double distance;
			double searchRatio;
			std::vector<Passage> path = arene_->pathFind(pos_, *but, Mesh::MinDist, distance, searchRatio);

			std::vector<Vec> buts;
			std::vector<double> cotes;

			for( auto it = path.rbegin(); it != path.rend(); it++)
			{
				buts.push_back((*it).coords);
				cotes.push_back((*it).sign);
			}
			ROS_INFO("Nouvel iti %d %.2f,%.2f %.2f,%.2f", (int)path.size(), pos_[0], pos_[1], (*but)[0], (*but)[1]);
			itineraire = Itineraire(buts,cotes);

		}
	}

	void PreBouge(double deltaT);

	void move(Vec pos)
	{
		pos_= pos;
		vertex_->move(pos,false);
	}


};

#endif /* SRC_ROSEBOT_H_ */
