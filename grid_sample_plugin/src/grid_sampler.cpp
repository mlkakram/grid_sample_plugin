#include "grid_sampler.h"

#include <Inventor/actions/SoGetBoundingBoxAction.h>

//for the bounding box and drawing forces
#include "graspit/graspitCore.h"
#include "graspit/body.h"
#include "graspit/robot.h"
#include "graspit/EGPlanner/searchState.h"
#include "graspit/ivmgr.h"


GridSampler::GridSampler(Hand *h, GraspableBody *b, double resolution):
    mHand(h),
    mObject(b),
    mResolution(resolution)
{
    //get object bbox dimensions
    SoGetBoundingBoxAction *bba =
        new SoGetBoundingBoxAction(graspitCore->getIVmgr()->getViewer()->getViewportRegion());
    bba->apply(mObject->getIVGeomRoot());
    SbVec3f bbmin,bbmax;
    bba->getBoundingBox().getBounds(bbmin,bbmax);
    delete bba;

    halfX = 0.5*(bbmax[0] - bbmin[0]);
    halfY= 0.5*(bbmax[1] - bbmin[1]);
    halfZ = 0.5*(bbmax[2] - bbmin[2]);
    transf object2BBoxCenter = transf::TRANSLATION(vec3(bbmin[0] + halfX, bbmin[1] + halfY, bbmin[2] + halfZ ));
    bboxCenterInWorld = mObject->getTran() % object2BBoxCenter;
}

void
BoxGridSampler::sample()
{
    //z from above
    //sampleFace( vec3(0, 1,0), vec3(-1,0,0), vec3(0,0,1) , halfX, halfZ, vec3(0,-halfY,0));
    //sampleFace( vec3(0,-1,0), vec3( 1,0,0), vec3(0,0,1) , halfX, halfZ, vec3(0, halfY,0));

    sampleFace( vec3(0,0, 1), vec3(0,1,0), vec3(-1,0,0) , halfY, halfX, vec3(0,0,-halfZ));
    sampleFace( vec3(0,0,-1), vec3(0,1,0), vec3( 1,0,0) , halfY, halfX, vec3(0,0, halfZ));

    sampleFace( vec3( 1,0,0), vec3(0, 1,0), vec3(0,0,1) , halfY, halfZ, vec3(-halfX,0,0));
    sampleFace( vec3(-1,0,0), vec3(0,-1,0), vec3(0,0,1) , halfY, halfZ, vec3( halfX,0,0));
}

void
BoxGridSampler::sampleFace(vec3 x, vec3 y, vec3 z,
                                double sz1, double sz2, vec3 tln)
{
    mat3 R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    int rotSamples = 1;


    double m1 = (2.0*sz1 - floor(2.0*sz1 / mResolution) * mResolution)/2.0;
    while (m1 < 2*sz1){
        double m2 = (2.0*sz2 - floor(2.0*sz2 / mResolution) * mResolution)/2.0;
        while (m2 < 2*sz2) {
            vec3 myTln(tln);
            myTln = myTln + (m1 - sz1)* y;
            myTln = myTln + (m2 - sz2)* z;
            transf tr(R, myTln);
            for(int rot=0; rot < rotSamples; rot++) {
                double angle = M_PI * ((double)rot) / rotSamples + (M_PI/2.0);
                transf rotTran = transf::AXIS_ANGLE_ROTATION(angle, vec3(1,0,0));
                std::cout << "rotTran: " << rotTran <<std::endl;
                tr = tr % rotTran;
                GraspPlanningState* seed = new GraspPlanningState(mHand);
                seed->setObject(mObject);
                seed->setRefTran(bboxCenterInWorld, false);
                seed->setPostureType(POSE_DOF, false);
                seed->setPositionType(SPACE_COMPLETE, false);
                seed->reset();
                seed->getPosition()->setTran(tr);
                mSamples.push_back(seed);
            }
            m2+=mResolution;
        }
        m1 += mResolution;
    }
}


void
EllipseSampler::sample()
{
    //generate a list of grasps by sampling an ellipsoid around the object
    GraspPlanningState seed(mHand);
    seed.setObject(mObject);

    //todo: should use bbox center as reference frame, not object origin
    //which could be anything
    seed.setRefTran(bboxCenterInWorld, false);
    seed.setPostureType(POSE_DOF, false);
    seed.setPositionType(SPACE_ELLIPSOID, false);
    seed.reset();

    //we don't want to sample distance
    seed.getPosition()->getVariable("dist")->setValue(0.0);
    seed.getPosition()->getVariable("dist")->setFixed(true);

    // set the dimension of the ellipse
    double size = 2.5;
    double aRes = size * halfX ;
    double bRes = size * halfY ;
    double cRes = size * halfZ ;

    seed.getPosition()->setParameter("a", aRes);
    seed.getPosition()->setParameter("b", bRes);
    seed.getPosition()->setParameter("c", cRes);

    //grid based sampling. Does somewhat better.
    gridEllipsoidSampling(seed);
}


/*! Samples an ellipsoid by sampling uniformly a grid with the same aspect
    ratio and projecting the resulting points on the ellipsoid. Not ideal,
    but at least much better then sampling angular variables directly */
void
EllipseSampler::gridEllipsoidSampling(const GraspPlanningState &seed)
{
    double step = (2*M_PI) / mResolution;

   double beta = 0;
   double tau = M_PI/2.0;
   for (double gamma=step; gamma<=(2*M_PI); gamma+=step) {
       addCartesianSamples(seed, beta, gamma, tau);
   }
}

void
EllipseSampler::addCartesianSamples(const GraspPlanningState &seed, double beta, double gamma, double tau)
{
        GraspPlanningState *newState = new GraspPlanningState(&seed);
        newState->getPosition()->getVariable("tau")->setValue(tau);
        newState->getPosition()->getVariable("gamma")->setValue(gamma);
        newState->getPosition()->getVariable("beta")->setValue(beta);
        mSamples.push_back(newState);
        std::cout << "new sample tran: " << newState->getPosition()->getCoreTran() << std::endl;
}



// void
// AboveSampler::sample()
// {
//     double step = (2*M_PI) / mResolution;
//     double approachDistance = halfZ ;

//     vec3 rotAxis = vec3(0,0,1); // rotate around Z axis
//     transf tr = transf(Quaternion(0,0,1,0), (halfZ+approachDistance) * rotAxis);

//     for (double roll=0; roll<(2*M_PI); roll+=step) {
//         std::cout << "step: \t" << roll/M_PI*180 <<std::endl;

//         // sample from top center rotating hand around roll
//         transf tr2 = transf::RPY(0, 0, roll);
//         tr2 = tr % tr2;

//         GraspPlanningState* seed = new GraspPlanningState(mHand);
//         seed->setObject(mObject);
//         seed->setRefTran(mObject->getTran(), false);
//         seed->setPostureType(POSE_DOF, false);
//         seed->setPositionType(SPACE_COMPLETE, false);
//         seed->reset();
//         seed->getPosition()->setTran(tr2);
//         mSamples.push_back(seed);
//     }
// }

void
AboveSampler::sample()
{
    //generate a list of grasps by sampling an ellipsoid around the object
    GraspPlanningState seed(mHand);
    seed.setObject(mObject);

    //todo: should use bbox center as reference frame, not object origin
    //which could be anything
    seed.setRefTran(bboxCenterInWorld, false);
    seed.setPostureType(POSE_DOF, false);
    seed.setPositionType(SPACE_ELLIPSOID, false);
    seed.reset();

    //we don't want to sample distance
    seed.getPosition()->getVariable("dist")->setValue(0.0);
    seed.getPosition()->getVariable("dist")->setFixed(true);

    // set the dimension of the ellipse
    double size = 2.5;
    double aRes = size * halfX ;
    double bRes = size * halfY ;
    double cRes = size * halfZ ;

    seed.getPosition()->setParameter("a", aRes);
    seed.getPosition()->setParameter("b", bRes);
    seed.getPosition()->setParameter("c", cRes);

    //grid based sampling. Does somewhat better.
    gridEllipsoidSampling(seed);
}


/*! Samples an ellipsoid by sampling uniformly a grid with the same aspect
    ratio and projecting the resulting points on the ellipsoid. Not ideal,
    but at least much better then sampling angular variables directly */
void
AboveSampler::gridEllipsoidSampling(const GraspPlanningState &seed)
{
    double step = (2*M_PI) / mResolution;

   double gamma = 0;
   double beta = M_PI/2.0;
   for (double tau=step; tau<=(2*M_PI); tau+=step) {
       addCartesianSamples(seed, beta, gamma, tau);
   }
}

void
AboveSampler::addCartesianSamples(const GraspPlanningState &seed, double beta, double gamma, double tau)
{
        GraspPlanningState *newState = new GraspPlanningState(&seed);
        newState->getPosition()->getVariable("tau")->setValue(tau);
        newState->getPosition()->getVariable("gamma")->setValue(gamma);
        newState->getPosition()->getVariable("beta")->setValue(beta);
        mSamples.push_back(newState);
        std::cout << "new sample tran: " << newState->getPosition()->getCoreTran() << std::endl;
}