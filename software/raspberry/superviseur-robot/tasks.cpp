/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30            
#define PRIORITY_TOPENCOMROBOT 20      
#define PRIORITY_TMOVE 23              //we set move to a higher priority than the task running the camera
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25    //very important task because we need to know the orders that come from the monitor in order to respond accordingly
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 15           //is less important, because checking the battery doesn't affect the behavior of the robot
#define PRIORITY_TRELOADWD 26          //very important, we need it to be on time or it will stop the robot 
#define PRIORITY_TOPENCAMERA 21
#define PRIORITY_TCLOSECAMERA 21
#define PRIORITY_TSEARCHARENA 20        






/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_battery, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_start, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_open_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_close_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_search_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_validate_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_search_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphores creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reloadWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************        return new Message(MESSAGE_ROBOT_POWEROFF);
************************************************************/
    
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_Battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_open_camera, "th_open_camera", 0, PRIORITY_TOPENCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_close_camera, "th_close_camera", 0, PRIORITY_TCLOSECAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_search_arena, "th_search_arena", 0, PRIORITY_TSEARCHARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;
    
    
    camera = new Camera();
    arenaFound = new Arena();
}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    //here we start the tasks that run all along, some wait for mutexes to be changed to start the task
    
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_open_camera, (void(*)(void*)) & Tasks::OpenCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::CameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_close_camera, (void(*)(void*)) & Tasks::CloseCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_search_arena, (void(*)(void*)) & Tasks::SearchArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }



    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {

        //we receive a message
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        //according to what type of message it is, we update the variables

        //if we receive a message saying we lost communication with the monitor:
        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);

            //we print a message to say we lost communication with the monitor
            cout << "Loss of the communication with the monitor\n" << msgRcv->ToString() << endl << flush;

            
            //stop robot
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            
            //close server
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Close();
            rt_mutex_release(&mutex_monitor);

            
            //close com with robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);

            //close camera
            rt_mutex_acquire(&mutex_close_camera, TM_INFINITE);
            close_camera = 1;  //when close_camera = 1, runs the task that closes the camera
            rt_mutex_release(&mutex_close_camera);

            exit(-1);
        } 
        //if we receive a message saying we want to open the communication with the robot:
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            
            //since we want to open the communication, we can release the semaphore 
            rt_sem_v(&sem_openComRobot);
        }
         //if we receive a message saying start the robot with or without watchdog:
         else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)||
                msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {

            //we get the id of the message to know if we start with or without a watchdog
            rt_mutex_acquire(&mutex_start, TM_INFINITE);
            start = msgRcv->GetID();
            rt_mutex_release(&mutex_start);

            //release the semaphore startRobot to start the StartRobotTask
            rt_sem_v(&sem_startRobot);

        } 
         //if we receive a message sayin how we should move: 
         else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);

        } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)) {
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            battery = 1; //tells to check battery
            rt_mutex_release(&mutex_battery);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_mutex_acquire(&mutex_open_camera, TM_INFINITE);
            open_camera = 1; //tells to open the camera
            rt_mutex_release(&mutex_open_camera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_mutex_acquire(&mutex_close_camera, TM_INFINITE);
            close_camera = 1; //tells to close the camera
            rt_mutex_release(&mutex_close_camera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_mutex_acquire(&mutex_search_arena, TM_INFINITE);
            search_arena = 1; //tells to search the arena
            rt_mutex_release(&mutex_search_arena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            rt_mutex_acquire(&mutex_validate_arena, TM_INFINITE);
            validate_arena = 1; //tells to validate the arena
            rt_mutex_release(&mutex_validate_arena);

            //enables the validation of the arena, only when we click on the button on the monitor
            rt_sem_v(&sem_arena);

        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_validate_arena, TM_INFINITE);
            validate_arena = 0; //tells to cancel arena
            rt_mutex_release(&mutex_validate_arena);

            //enables the infirmation of the arena, only when we click on the button on the monitor
            rt_sem_v(&sem_arena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_mutex_acquire(&mutex_search_robot, TM_INFINITE);
            searchRobot = 1; //tells to start looking for robot on picture
            rt_mutex_release(&mutex_search_robot);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_mutex_acquire(&mutex_search_robot, TM_INFINITE);
            searchRobot = 0; //tells to stop looking for robot on picture
            rt_mutex_release(&mutex_search_robot);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        Message * msgSend;
        //we wait until we are allowed to start the robot
        rt_sem_p(&sem_startRobot, TM_INFINITE);

        //with the ID, we put in start, we check if we want to start with or without the watchdog
        if(start == MESSAGE_ROBOT_START_WITHOUT_WD) {
            
            cout << "Start robot without watchdog";

            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());

            //we use the counter_write after each time we call Write
            if (msgSend -> CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                counter_write ++; //we increment the timer every time we fail to communicate with the robot
                if (counter_write >=3) { //if we fail 3 times then we stop the robot
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                   
                }
            }
            else { //if we managed to communicate then we reset the counter
                counter_write = 0;
            }
            rt_mutex_release(&mutex_robot);

            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) { //if we successfully started the robot we set robotStarted to 1
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            } else if (msgSend->GetID() == MESSAGE_ANSWER_NACK) { //we set robotStarted to 0 because we failed to start the robot
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
            }
        } 
        
        else if(start == MESSAGE_ROBOT_START_WITH_WD) {
            /* Send reload watchdog every second
            Before the old watchdog expires (after 1s), the next watchdog has to be sent within a range of 50ms. 
            Therefore, the start of the robot has to be synchronized with the start of the watchdog task. That is why we start the task reloadWD from the start robot with the watchdog and not from the task Init().
            */
            cout << "Start robot with watchdog (";

            

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);

            msgSend = robot.Write(robot.StartWithWD());

            
            if (msgSend -> CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                counter_write ++;
                if (counter_write >=3) {
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                }
            }
            else {
                counter_write = 0;
            }

            rt_mutex_release(&mutex_robot);

            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            } else if (msgSend->GetID() == MESSAGE_ANSWER_NACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
            }

            rt_sem_v(&sem_reloadWD); //release the semaphore for synchronization between the counter of the robot and the watchdog

        } 
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            cout << "Periodic movement update";

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);

            Message * msgSend = robot.Write(new Message((MessageID)cpMove));
            
            
            if (msgSend -> CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                counter_write ++;
                if (counter_write >=3) {
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                }
            }
            else {
                counter_write = 0;
            }
            

            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}



/**
 * @brief Thread handling reloading the watchdog of the robot.
 */
void Tasks::ReloadWDTask(void *arg) {
    int rs;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    //semaphore watchdog to synchronise with the counter of the robot
    rt_sem_p(&sem_reloadWD, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Reloading movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
                       
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);

            Message * msgSend = robot.Write(new Message (MESSAGE_ROBOT_RELOAD_WD)); //message to reload the watchdog
            if (msgSend -> CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                counter_write ++;
                if (counter_write >=3) {
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                }
            }
            else {
                counter_write = 0;
            }

            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}



/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}



/**
 * @brief Thread handling control of the robot.
 */
void Tasks::BatteryTask(void *arg) {
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000); //we check every 100 ms

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { 

            if (battery==1) { //if we want to read the battery
               
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                Message * battery_level_message = robot.Write(robot.GetBattery()); //get the battery
                rt_mutex_release(&mutex_robot);

                if (battery_level_message -> CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                    counter_write ++;
                    if (counter_write >=3) {
                        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                        robotStarted = 0;
                        rt_mutex_release(&mutex_robotStarted);
                    }
                }
                else {
                    counter_write = 0;
                }

                rt_mutex_acquire(&mutex_battery, TM_INFINITE);
                battery=0; //we don't want to read the battery level anymore
                rt_mutex_release(&mutex_battery);   

                cout << " battery level ";

                WriteInQueue(&q_messageToMon, battery_level_message); 
  
            }

        }
        cout << endl << flush;
    }
}


void Tasks::OpenCameraTask(void *arg) {
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { 

            if (open_camera==1) { //if we want to open the camera
               
                if (camera -> Open()) { //opens camera
                    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                    runningCamera = 1; //tells to do the task running camera
                    rt_mutex_release(&mutex_camera);

                } else {
                    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR)); 
                    cout << "We failed to open the camera";
                };

                rt_mutex_acquire(&mutex_open_camera, TM_INFINITE);
                open_camera=0;
                rt_mutex_release(&mutex_open_camera);   

            }

        }
        cout << endl << flush;
    }
}


void Tasks::CameraTask(void *arg) {
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { //si le robot est allumé

            if (runningCamera==1) {
               
                Img image = camera -> Grab();

                if (searchRobot==1) {
                    
                   std::list<Position> listPosition = image.SearchRobot(*arenaFound);

                   /*this is the message with position (-1, -1)
                   if we didn't find any robot, this is the message we return
                   */
                   
                
                   if (!listPosition.empty()){
                       image.DrawAllRobots(listPosition);
                    

                       for (Position robot : listPosition) {

                           MessagePosition * message = new MessagePosition(MESSAGE_CAM_POSITION, robot);
                           WriteInQueue(&q_messageToMon, message); // message will be deleted by sendToMon
                       } 
                   }
                   else {
                        Position * position = new Position();
                        MessagePosition * message = new MessagePosition(MESSAGE_CAM_POSITION, *position);
                       WriteInQueue(&q_messageToMon, message); // message will be deleted by sendToMon
                    }

                }

                if (!(arenaFound -> IsEmpty())) {
                    image.DrawArena(*arenaFound);
                }


                MessageImg * message = new MessageImg(MESSAGE_CAM_IMAGE, &image);
                WriteInQueue(&q_messageToMon, message); 

                
                
            }


        }
        cout << endl << flush;
    }
}


void Tasks::CloseCameraTask(void *arg) {
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { 

            if (close_camera==1) { //if we want to close the camera
                
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                runningCamera = 0; //tells to stop the task running camera
                rt_mutex_release(&mutex_camera);
                camera -> Close(); //we close the camera
                
                Message * msgSend;
                if (camera -> IsOpen()) { //if the camera is still open : problem
                    msgSend = new Message(MESSAGE_ANSWER_NACK);
                    cout << "We failed to close the camera";
                } else {
                    msgSend = new Message(MESSAGE_ANSWER_ACK); //we send a message of confirmation
                }
                WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

                rt_mutex_acquire(&mutex_close_camera, TM_INFINITE);
                close_camera=0;
                rt_mutex_release(&mutex_close_camera);   

            }

        }
        cout << endl << flush;
    }
}


void Tasks::SearchArenaTask(void *arg) {
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { 

            if (search_arena==1) { //if we want to search for the arena
                
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                runningCamera = 0; //tells to stop the task running camera
                rt_mutex_release(&mutex_camera);
                

                Img image = camera -> Grab(); //we capture the image of the camera
                //MessageImg * message = new MessageImg(MESSAGE_CAM_IMAGE, &image);

                Arena anArena = image.SearchArena(); //we look for the arena

                Message * msgSend;
                if (anArena.IsEmpty()) { //we didn't find the arena
                    msgSend = new Message(MESSAGE_ANSWER_NACK); 
                    cout << "We couldn't find the arena";
                } else {
                    image.DrawArena(anArena); //if we found the arena, then we draw it
                    msgSend = new Message(MESSAGE_ANSWER_ACK);

                    MessageImg * messageImageArena = new MessageImg(MESSAGE_CAM_IMAGE, &image);
                    WriteInQueue(&q_messageToMon, messageImageArena); //we send the image to the monitor


                    rt_sem_p(&sem_arena, TM_INFINITE); //semaphore to wait until the button for validation is clicked

                    if (validate_arena == 1) { //if the arena is ok
                        rt_mutex_acquire(&mutex_arena_found, TM_INFINITE);
                        *arenaFound = anArena; //set the arena found to the arenaFound variable to be able to use it when the camera is running
                        rt_mutex_release(&mutex_arena_found);
                        
                        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                        runningCamera = 1; //the camera can start again
                        rt_mutex_release(&mutex_camera);
                    } else if (validate_arena == 0) {   //if the arena is not ok we don't draw it and go back to starting the camera                    
                        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                        runningCamera = 1; 
                        rt_mutex_release(&mutex_camera);
                    }
                    
                    rt_mutex_acquire(&mutex_validate_arena, TM_INFINITE);
                    validate_arena = -1;
                    rt_mutex_release(&mutex_validate_arena);

                    rt_sem_v(&sem_arena); //we release the semaphore so that it waits until someone clicks on the button


                }
                WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon


                rt_mutex_acquire(&mutex_search_arena, TM_INFINITE);
                search_arena=0;
                rt_mutex_release(&mutex_search_arena);   

                

            }

        }
        cout << endl << flush;
    }
}

