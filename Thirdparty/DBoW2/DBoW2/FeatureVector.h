/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include "BowVector.h"
#include <map>
#include <vector>
#include <iostream>

//#include <boost/serialization/serialization.hpp>
//#include <boost/serialization/map.hpp>

namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector: 
  //NodeId-节点id，std::vector<unsigned int> -当前node下的特征点索引
  public std::map<NodeId, std::vector<unsigned int> >
{
//    friend class boost::serialization::access;
//    template<class Archive>
//    void serialize(Archive& ar, const int version)
//    {
//        ar & boost::serialization::base_object<std::map<NodeId, std::vector<unsigned int> > >(*this);
//    }

public:
  typedef std::map<NodeId, std::vector<unsigned int> > super;

  /**
   * Constructor
   */
  FeatureVector(void);
  
  /**
   * Destructor
   */
  ~FeatureVector(void);
  
  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(NodeId id, unsigned int i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
    
};

} // namespace DBoW2

#endif

