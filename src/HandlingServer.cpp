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
        cout << "Robot offline." << endl;
        break;

    case 1: // BCO completed, ready

        loadTrajectories(); // load trajectories

        if (!infoQueue.empty() && !trajectoryQueue.empty()) // some success in loading
        {
            sendTrajectory(infoQueue.front(),trajectoryQueue.front());    // send the first one
            infoQueue.pop();
            trajectoryQueue.pop();
        }

        break;

    case 2: // Received trajectory, about to move

        // start capturing frames from robot
        capturedFrames.clear();
        capturedFrames.push_back(response.frame);

    case 3: // Executing trajectory, moving

        // continue capturing frames from robot
        capturedFrames.push_back(response.frame);

    case 4: // Finished trajectory, ready for next

        // end capturing frames from robot
        capturedFrames.push_back(response.frame);

        // write captured to file
        cout << "----------------------------------" << endl;
        cout << "Saving trajectory id " << response.info[1] << endl;
        std::string captureName = (boost::format("captured_%1%.txt") %response.info[1]).str();
        cout << "Filename:  " << captureName << endl;
        cout << "----------------------------------" << endl;
        try
        {
            std::ofstream captureFile;
            captureFile.open( captureName );

            for (auto frame : capturedFrames)
            {
                for (auto val : frame)
                {
                    captureFile << val << " ";
                }
                captureFile << "\n";
            }

        }
        catch (exception &e)
        {
            cerr << "Failed writing captured data to file: " << e.what();
        }

        // send next available trajectory
        // last trajectory has running = 0 so robot will exit

        if (!infoQueue.empty() && !trajectoryQueue.empty())
        {
            sendTrajectory(infoQueue.front(),trajectoryQueue.front());    // send the first one
            infoQueue.pop();
            trajectoryQueue.pop();
        }

        break;
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

    int filecount = boost::lexical_cast<int>(filenames.size()); // the lexical_cast eliminates compiler warning

    // start assigning id's, arbitrary, let's define id=0 as "non-existent"
    int trajectoryId = 1;

    // load and enqueue each file
    for (auto file : filenames)
    {
        trajectory_vec trajectory;
        dataFile.loadSDFrames(file, trajectory);  // load a file and store contents in trajectory vector

        if (!trajectory.empty())    // some success in loading is required
        {
            // each trajectory gets an id
            trajInfo[2] = trajectoryId;
            trajInfo[4] = 200;  // velocity 200 for all except...

            // let's do double velocity on fourpoints
            if (trajectoryId == 2)
            {
                trajInfo[4] = 400;
            }

            // if last trajectory, set running to 0 so robot exits
            if (trajectoryId == filecount)
            {
                trajInfo[3] = 0;
            }

            // **required** set the framecount
            // here assuming each frame in trajectory_vec is correct, we use size of trajectory vector
            trajInfo[6] = boost::lexical_cast<int>(trajectory.size());  // the lexical_cast eliminates compiler warning

            // enqueue info and trajectory
            trajectoryQueue.push(trajectory);
            infoQueue.push(trajInfo);

            ++trajectoryId;
        }
    }

}
