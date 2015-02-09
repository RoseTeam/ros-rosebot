/*
 * itineraire.h
 *
 *  Created on: Feb 7, 2015
 *      Author: gg99
 */

#ifndef SRC_ITINERAIRE_H_
#define SRC_ITINERAIRE_H_
#include <vector>
#include "point2d.h"
#include <ros/ros.h>

typedef Point2d Vec;

class Itineraire
{
	std::vector<Point2d> posns_;
	std::vector<double> cotes_;
	int index_;

public:

	Itineraire(){index_=0;};
	Itineraire(std::vector<Point2d> buts, std::vector<double> cotes):posns_(buts), cotes_(cotes){
		index_ = -posns_.size();
	}

	void ajout(Point2d pos, double cote=0, bool enTete=true)
	{
		if( enTete && index_ != 0 )
		{
			posns_.insert(posns_.begin()+index_,pos);
			cotes_.insert(cotes_.begin()+index_,cote);
		}
		else
		{
			posns_.push_back(pos);
			cotes_.push_back(cote);
		}
		index_ -= 1;
	}

	void increme(){
		index_ += 1;
	}

    void maj_pos(double rayon, Vec* prox, double&  rpivot, Vec* suiv, double& rpivot_suiv)
    {
    	prox = nullptr;
    	suiv = nullptr;
    	rpivot_suiv = 0;
    	rpivot = 0;
        if( index_ )
		{

            double marge = rayon * 1.5;
            rpivot = cotes_[index_+cotes_.size()] * .5;
            if (fabs(rpivot) > marge)
                rpivot = rpivot > 0 ? marge : -marge;

            prox = &posns_[index_+posns_.size()];
            rpivot_suiv = cotes_[index_+1+cotes_.size()] * .5;
            if(abs(rpivot_suiv) > marge)
                rpivot_suiv = rpivot_suiv > 0 ? marge : -marge;

            suiv = & posns_[index_+1+posns_.size()];

		}
    }

};

#endif /* SRC_ITINERAIRE_H_ */
