#ifndef HANDLINGSERVER_H
#define HANDLINGSERVER_H

#include "kuros.h"


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

    std::vector< trajectory_vec > trajectoryQueue;

    trajectory_vec capturedFrames;

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
    Loads and enqueues trajectories in trajectoryQueue.
    */
    void loadTrajectories();

    /*
    Saves capturedFrames to a space delimited file.
    */
    void saveCaptured();


};

#endif // HANDLINGSERVER_H
