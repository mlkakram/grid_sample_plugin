#include "grid_sampler.h"

#include <Inventor/actions/SoGetBoundingBoxAction.h>

//for the bounding box and drawing forces
#include "graspit_source/include/graspitCore.h"
#include "graspit_source/include/body.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/EGPlanner/searchState.h"
#include "ivmgr.h"


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

    //grid based sampling. Does somewhat better.
    gridEllipsoidSampling(seed);
}


/*! Samples an ellipsoid by sampling uniformly a grid with the same aspect
    ratio and projecting the resulting points on the ellipsoid. Not ideal,
    but at least much better then sampling angular variables directly */
void
EllipseSampler::gridEllipsoidSampling(const GraspPlanningState &seed)
{
    double aRes = 2.0 * halfX ;
    double bRes = 2.0 * halfY ;
    double cRes = 2.0 * halfZ ;

    double step = (2*M_PI) / mResolution;

    for (double roll=step; roll<=(2*M_PI); roll+=step) {

        double x = cos(roll);
        double y = sin(roll);
        addCartesianSamples(seed, x*aRes, y*bRes , 0);
    }
}

void
EllipseSampler::addCartesianSamples(const GraspPlanningState &seed, double x, double y, double z)
{
    //double c = seed.readPosition()->getParameter("c");
    //compute angular values
    //from HandObjectStateImpl:
    //x =  a * cos(beta) * cos(gamma);
    //y =  b * cos(beta) * sin(gamma);
    //z =  c * sin(beta);
    double beta = asin(z / sqrt(x*x + y*y + z*z));
    double gamma = atan2(y/halfY, x/halfX);

    //sample roll angle as well
    //for (int m=0; m<mResolution; m++) {
        //only sample from 0 to almost PI, as the HH is symmetric
        double tau = M_PI/2.0;//* ((double)m) / mResolution;
        GraspPlanningState *newState = new GraspPlanningState(&seed);
        newState->getPosition()->getVariable("tau")->setValue(tau);
        newState->getPosition()->getVariable("gamma")->setValue(gamma);
        newState->getPosition()->getVariable("beta")->setValue(beta);
        mSamples.push_back(newState);
        std::cout << "new sample tran: " << newState->getPosition()->getCoreTran() << std::endl;
    //}
}

