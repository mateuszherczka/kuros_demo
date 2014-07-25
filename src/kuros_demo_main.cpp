#include <iostream>

#include <HandlingServer.hpp>   // implemented by user

int main()
{

    // start server
    HandlingServer aserver;     // loads config file on creation

    // connect
    aserver.startListening();   // blocks until connection

    // from now on the HandlingServer deals with the robot.

    // receive ready response from robot

    // load trajectories from files

    // enqueue trajectories

        // send 1st traj

        // receive next ready response

        // send 2d traj

        // receive next ready ... etc

    // send last "home" trajectory with a running=0

    // exit

    cout << "Hello world!" << endl;
    return 0;
}
