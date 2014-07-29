#ifndef HANDLINGSERVER_H
#define HANDLINGSERVER_H

#include <kuros.h>
#include <DataFile.hpp>
#include <queue>
#include <string>

#include <boost/format.hpp>


class HandlingServer : public Server
{

public:
    HandlingServer();
    virtual ~HandlingServer();

    /*
    Handles incoming messages from robot.
    We have access to response object in server.
    */
    void handleResponse() override;
    void handleDisconnect() override;

protected:
private:

    //---------------------------------------------------------
    // Data
    //---------------------------------------------------------

    int handledCount = 0;
    int streamedResponseCount = 0;

    /*
    trajectory_vec is a vector of frames, a "trajectory".
    info_vec is a vector of int parameters for a trajectory.
    */
    std::queue< trajectory_vec > trajectoryQueue;
    std::queue< info_vec > infoQueue;

    trajectory_vec capturedFrames;

    // trajectory made in matlab
    std::string trajectoryFile { "trajectory.txt" };

    /*
    Response modes:

    1   response when done with BCO, about to run a trajectory, done with a trajectory, exiting
    2   work in progress
    3   mode 1 + stream of responses every N ms all the time

    */

    // lets define default trajectory parameters (integers)
    info_vec trajInfo { 3,       // response mode
                        20,      // response stream interval N ms (probably 12ms is the smallest possible)
                        1000,    // trajectory id, returned by robot when running trajectory
                        1,       // 1 = keep running, 0 = exit after finishing trajectory
                        200,     // velocity [mm/s], 200 is a comfortable number, max is around 2000
                        20,      // distance [mm] when robot is allowed to start approximating a point
                        1       // frame type, 1 = cartesian, 2 = joint axis
                      };

    /*
    Robot status as sent by the robot:

    0   not connected or exiting
    1   ready for session, 1st message from robot after BCO
    2   received trajectory, about to start moving
    3   executing trajectory, moving
    4   finished trajectory, ready for next one

    */
    int robotStatus = 0;

    //---------------------------------------------------------
    // Methods
    //---------------------------------------------------------

    /*
    Loads and enqueues trajectories.
    */
    void loadTrajectories();

    /*
    Saves capturedFrames to a space delimited file.
    */
    void saveCaptured();


};

#endif // HANDLINGSERVER_H
