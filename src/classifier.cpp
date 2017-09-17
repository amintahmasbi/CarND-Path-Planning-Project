#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <set>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {
  priors.resize(possible_labels.size());
  Means.resize(possible_labels.size());
  Variances.resize(possible_labels.size());
}

GNB::~GNB() {}

template <typename T>
T normal_pdf(T x, T m, T s)
{
  static const T inv_sqrt_2pi = 0.3989422804014327;
  T a = (x - m) / s;

  return inv_sqrt_2pi / s * exp(-T(0.5) * a * a);
}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

  /*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
   */

  //Extract unique labels
  //  set<string> unique_labels_set;
  //  unsigned size = labels.size();
  //  for( unsigned i = 0; i < size; ++i ) unique_labels_set.insert( labels[i] );
  //
  //  vector<string> unique_labels;
  //  unique_labels.assign( unique_labels_set.begin(), unique_labels_set.end() );

  //Count number of each labels
  vector<vector<int>> indexes(possible_labels.size());

  for(vector<string>::size_type i = 0; i != possible_labels.size(); i++)
  {
    for(vector<string>::size_type j = 0; j != labels.size(); j++)
    {
      if (labels[j].compare(possible_labels[i]) == 0)
      {
        indexes[i].push_back(j);
      }
    }
  }

  //Calculate prior priorities
  for(vector<double>::size_type i = 0; i != priors.size(); i++)
  {
    priors[i] = double(indexes[i].size()) / double(labels.size());
  }


  //Calculate mean and variance per feature
  for(vector<int>::size_type i = 0; i != indexes.size(); i++)
  {
    //Mean
    vector<double> total(data[0].size(),0.0);
    for(vector<int>::size_type j = 0; j != indexes[i].size(); j++)
    {
      int idx = indexes[i][j];
      for (int k = 0; k < total.size(); ++k)
      {
        total[k] += data[idx][k];

      }

    }


    for (int n = 0; n < total.size(); ++n)
    {
      double tmp_mean = total[n]/double(indexes[i].size());
      Means[i].push_back(tmp_mean);
    }

    //Variance
    vector<double> var_total(data[0].size(),0.0);
    for(vector<int>::size_type j = 0; j != indexes[i].size(); j++)
    {
      int idx = indexes[i][j];
      for (int k = 0; k < total.size(); ++k)
      {
        var_total[k] += ( data[idx][k]-Means[i][k]) * ( data[idx][k]-Means[i][k]);
      }
    }

    for (int n = 0; n < var_total.size(); ++n)
    {
      double tmp_var = var_total[n]/(indexes[i].size()-1);
      Variances[i].push_back(tmp_var);
    }

  }



}

string GNB::predict(vector<double> data_point)
{
  /*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
   */
  vector<double> posteriors(possible_labels.size());

  for(vector<double>::size_type i = 0; i != priors.size(); i++)
  {
    posteriors[i] = priors[i];

    for (int k = 0; k < data_point.size(); ++k)
    {
      posteriors[i] *= normal_pdf(data_point[k],Means[i][k],sqrt(Variances[i][k]));
    }
  }

  int best_label = distance(posteriors.begin(), max_element(posteriors.begin(), posteriors.end()));
  cout << best_label << endl;
  return this->possible_labels[best_label];

}
