#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    Debugger::init();

    // queue = xQueueCreate(queueSize, sizeof(a010_frame_t));
    // if (queue == NULL)
    {
        //      Debugger::log("Error creating the queue", ERROR);
    }

    a010 = A010();
    delay(500);

    // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(Task1code, /* Task function. */
                            "Task1",   /* name of task. */
                            100000,    /* Stack size of task */
                            NULL,      /* parameter of the task */
                            10,        /* priority of the task */
                            &Task1,    /* Task handle to keep track of created task */
                            0);        /* pin task to core 0 */

    // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(Task2code, /* Task function. */
                            "Task2",   /* name of task. */
                            10000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            5,         /* priority of the task */
                            &Task2,    /* Task handle to keep track of created task */
                            1);        /* pin task to core 1 */
}

void loop()
{
    // Do not put code in the loop() when using freeRTOS.
    // loop() is the only task that is guaranteed to not be ran per tasking iteration.
    delay(1000);
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task1code(void *pvParameters)
{
    a010_frame_t a010Packet;

    uint16_t i, result;
    std::vector<ClusterPoint3D> cloud;
    std::vector<std::vector<uint16_t>> clusters;

    while (1)
    {
        if (a010.ReadSerial())
        {
            // a010.CheckContinuity();

            // std::vector<std::vector<float>> w;
            float w[PICTURE_SIZE][3]{0};

            // SERIAL_DEBUG.println("vector capacity: " + String(w.capacity()));  // vector capacity: 0
            // SERIAL_DEBUG.println("vector max size: " + String(w.max_size()));  // vector max size: 357913941

            // w.reserve(2500);

            for (i = 0; i < PICTURE_SIZE; i++)  // FIXME: n'affiche plus rien !
            {
                // std::vector<float> v;
                // v.push_back(cloud[i].x);
                // v.push_back(cloud[i].y);
                // v.push_back(cloud[i].z);
                // w.push_back(v);
                w[i][0] = a010.cloudFrame[i].x;
                w[i][1] = a010.cloudFrame[i].y;
                w[i][2] = a010.cloudFrame[i].z;
                String data = "" + String(a010.cloudFrame[i].x) + " " + String(a010.cloudFrame[i].y) + " " + String(a010.cloudFrame[i].z) + " " +
                              String("65520");
                SERIAL_DEBUG.println(data);
            }
            SERIAL_DEBUG.println("-----------");

            // SERIAL_DEBUG.println("vector capacity: " + String(w.capacity()));  // vector capacity: 512
            // SERIAL_DEBUG.println("vector max size: " + String(w.max_size()));
            // SERIAL_DEBUG.println("vector size: " + String(w.size()));  // vector size: 300
            //  SERIAL_DEBUG.println("vector shrink to fit : " + w.shrink_to_fit());
            //  SERIAL_DEBUG.println("vector size: " + w.size());

            // dbscan(epsilon, minPts, distance, mink)
            // dbscan DB(30, 10, EUCLIDIAN);
            SERIAL_DEBUG.println("dbscan setup");
            // clusters = DB.init(w);
            // DB.init(w);
            SERIAL_DEBUG.println("dbscan init");
            // SERIAL_DEBUG.println(clusters.size() - 1);
            // DB.displayStats();

            //+" " + String(cloud.cluster[i]);
            // SERIAL_DEBUG.println(String(dist));

            // TODO: afficher les clusters en changeant la couleur dans fichier PCD

            // xQueueSend(queue, &a010Packet, 0);
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    a010_frame_t a010Packet;
    uint16_t distance_mm = 0;
    Point3D p = {0, 0, 0};
    while (1)
    {
        // if (uxQueueMessagesWaiting(queue) > 0)
        {
            //   if (xQueueReceive(queue, &a010Packet, portTICK_PERIOD_MS * 0))
            {
                /*
                for (int x = 1; x <= 25; x++)
                {
                    for (int y = 1; y <= 25; y++)
                    {
                        p.x = x;
                        p.y = y;
                        p.z = a010Packet.payload[x + (y - 1) * 25] * QUANTIZATION_MM;
                        // Debugger::plot3D(p, "p" + String(x) + "_" + String(y));
                        Debugger::plot3Dpy(p);
                    }
                }*/
                //       a010.GetPointCloudFromFrame(a010Packet);
            }
        }

        // Check if we get commands from operator via debug serial
        String cmd = Debugger::checkSerial();

        if (cmd != "")
        {
            if (cmd.startsWith("cmdAT:"))
            {
                try
                {
                    cmd.remove(0, 6);
                    // int i = atoi(cmd.c_str());
                    Debugger::log("cmdAT: ", cmd);
                    SERIAL_A010.write(cmd.c_str());
                }
                catch (std::exception const &e)
                {
                    Debugger::log("error : ", e.what());
                }
            }
        }

        vTaskDelay(1);
    }
}
