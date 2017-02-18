#include <vector>
#include "graspit_source/include/matvec3D.h"

class GraspPlanningState;
class GraspableBody;
class Hand;

//! Base Class for sampling hand configurations from around a model
class GridSampler{

public:

    GridSampler(Hand* h, GraspableBody *b, double resolution);
    //!
    virtual void sample() = 0;

    std::vector<GraspPlanningState*>* getResults(){return &mSamples;}


protected:

    //!list of found hand states
    std::vector<GraspPlanningState*> mSamples;
    //!object we are sampling
    GraspableBody* mObject;

    Hand* mHand;

    transf bboxCenterInWorld;

    //! sampling resolution
    double mResolution;

    double halfX;
    double halfY;
    double halfZ;
};


class BoxGridSampler : public GridSampler
{

public:

    BoxGridSampler(Hand* h, GraspableBody *b, double resolution):
        GridSampler(h, b, resolution)
    {

    }
    void sample();

protected:

    void sampleFace(vec3 x, vec3 y, vec3 z, double sz1, double sz2, vec3 tln);
};

class EllipseSampler : public GridSampler
{

public:

    EllipseSampler(Hand* h, GraspableBody *b, double resolution):
        GridSampler(h, b, resolution)
    {

    }

    void sample();

protected:

    //! Helper function for the above
    void addCartesianSamples(const GraspPlanningState &seed, double x, double y, double z);

    //! Samples an ellipsoid using a grid-based method to generate pre-grasps.
    void gridEllipsoidSampling(const GraspPlanningState &seed);
};
