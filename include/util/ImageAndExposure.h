/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include <cstring>
#include <iostream>

#include <util/akaze/AKAZE.h>
#include <util/akaze/nldiffusion_functions.h>

namespace dso
{


class ImageAndExposure
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	float* image;			// irradiance. between 0 and 256
  std::vector<cv::Mat> imgMats;
	int w,h;				// width and height;
	double timestamp;
	float exposure_time;	// exposure time in ms.
	inline ImageAndExposure(int w_, int h_, double timestamp_=0) : w(w_), h(h_), timestamp(timestamp_)
	{
		image = new float[w*h];
		exposure_time=1;
	}
	inline ~ImageAndExposure()
	{
		delete[] image;
	}

	inline void copyMetaTo(ImageAndExposure &other)
	{
		other.exposure_time = exposure_time;
	}

  void create_undistMat()
  {
    cv::Mat imMat = cv::Mat(w,h,CV_32FC1,image);
    imgMats.push_back(imMat);
    for (int i = 1 ; i < 3 ; i++)
    {
      cv::Mat imMatG;
      gaussian_2D_convolution(imMat,imMatG,0,0,0.5*i);
      imgMats.push_back(imMatG);
    }
  }

	inline ImageAndExposure* getDeepCopy()
	{
		ImageAndExposure* img = new ImageAndExposure(w,h,timestamp);
		img->exposure_time = exposure_time;
		memcpy(img->image, image, w*h*sizeof(float));
		return img;
	}
  AKAZEOptions kazeoptions;

};


}
