#include "HandlingServer.hpp"

HandlingServer::HandlingServer() {}

HandlingServer::~HandlingServer() {}

void HandlingServer::handleResponse()
{
    /*
    Response info vector:

    index   message
    ---------------
    0       Status
    1       Trajectory Id
    2       Tick (sent time)

    */

    robotStatus = response.info[0];

    switch (robotStatus)
    {
        case 0: // not connected or exiting

        case 1: // BCO completed, ready

            // send first trajectory

        case 2: // Received trajectory, about to move

        case 3: // Executing trajectory, moving

        case 4: // Finished trajectory, ready for next

            // send next available trajectory
            // last trajectory will have running = 0
    }


    cout << "------------------------------------------------" << endl;
    cout << "Response #" << handledCount << endl;

    response.printValues();

    cout << "------------------------------------------------" << endl;

    ++handledCount;
}

void HandlingServer::handleDisconnect()
{
    // we can call startListening() again
    cout << "Server disconnected." << endl;
}

void HandlingServer::loadTrajectories()
{
    // helper for loading
    DataFile dataFile;

    /*
    Response modes:

    1   response when done with BCO, about to run a trajectory, done with a trajectory, exiting
    2   mode 1 + stream of responses every N ms while running a trajectory (stream stops when trajectory done)
    3   mode 1 + stream of responses every N ms all the time

    */

    // lets define default trajectory parameters (integers)
    info_vec trajInfo { 2       // response mode
                        20      // response stream interval N ms (probably 12ms is the smallest possible)
                        1       // trajectory id, returned by robot when running trajectory
                        1       // 1 = keep running, 0 = exit after finishing trajectory
                        200     // velocity [mm/s], 200 is a comfortable number, max is around 2000
                        20      // distance [mm] when robot is allowed to start approximating a point
                        1       // framecount in trajectory, **very important to be correct**
                      };

    // trajectories made in matlab
    std::vector <std::string> filenameQueue { "spiral.txt", "fourpoints.txt", "home.txt" };

    // load and enqueue frames and infos
    int trajectoryId = 1;
    for (auto filename : filenameQueue)
    {
        trajectory_vec trajectory;
        dataFile.loadSpaceDelimited(filename, trajectory);  // store file contents in trajectory vector

        trajectoryQueue.push_back(trajectory);

        // each trajectory gets an id
        trajInfo[2] = trajectoryId;

        // if last trajectory, set running to 0
        if (trajectoryId == boost::lexical_cast<int>filenameQueue.size())
        {
            trajInfo[3] = 0;
        }

        // set the framecount, here assuming each frame in trajectory_vec is correct (a weak assumption)
        trajInfo[6] = boost::lexical_cast<int>(trajectory.size());

        trajectoryInfoQueue.push_back(trajInfo);

        ++trajectoryId;
    }

}
