#include "configurator.h"

#include <sstream>
#include <atom/osc.h>

/*************/
Configurator::Configurator():
    mReady(false),
    mVerbose(false),
    mOscServer(NULL)
{
    mLastIndexReceived = 0;
}

/*************/
Configurator::~Configurator()
{
    if (mOscServer != NULL)
    {
        lo_server_thread_stop(mOscServer);
        lo_server_thread_free(mOscServer);
    }
}

/*************/
void Configurator::loadXML(const char* filename)
{
    cout << "Attempting to read XML file " << filename << endl;

    xmlDocPtr doc;
    doc = xmlReadFile(filename, NULL, 0);

    if (doc == NULL)
    {
        cout << "Failed to parse " << filename << endl;
        return;
    }

    xmlNodePtr cur;
    cur = xmlDocGetRootElement(doc);
    if (cur == NULL || cur->xmlChildrenNode == NULL)
    {
        cout << "Document seems to be empty" << endl;
        return;
    }

    cur = cur->xmlChildrenNode;

    while (cur != NULL)
    {
        if (!xmlStrcmp(cur->name, (const xmlChar*)"Flow"))
        {
            bool error = loadFlow(doc, cur);
            if (error)
            {
                cout << "An error has been detected while parsing file " << filename << endl;
            }
        }

        cur = cur->next;
    }
}

/*************/
bool Configurator::loadFlow(const xmlDocPtr doc, const xmlNodePtr cur)
{
    bool error = false;
    xmlNodePtr lCur;
    
    timespec waitTime;
    waitTime.tv_sec = 0;
    waitTime.tv_nsec = 1e6;

    struct simpleFlow
    {
        int id;
        string address;
        int port;
    } simpleFlow;

    shared_ptr<OscClient> address; 

    string nullString = string("");

    if (cur->xmlChildrenNode == NULL)
        error = true;
    else
    {
        lCur = cur->xmlChildrenNode;

        // We need to get the detector name, as well as information about sources
        // to create the flow (before any change of parameters)
        string detector;
        vector<string> sources;
        vector<int> subsources;
        string client, server;
        string serverPort;
        int clientPort;

        while (lCur != NULL)
        {
            if (!xmlStrcmp(lCur->name, (const xmlChar*)"Detector"))
            {
                detector = getStringValueFrom(doc, lCur, (const xmlChar*)"Type");
            }
            else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Source"))
            {
                sources.push_back(getStringValueFrom(doc, lCur, (const xmlChar*)"Type"));
                subsources.push_back(getIntValueFrom(doc, lCur, (const xmlChar*)"Subsource"));
            }
            else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Client"))
            {
                client = getStringValueFrom(doc, lCur, (const xmlChar*)"Address");
                clientPort = getIntValueFrom(doc, lCur, (const xmlChar*)"Port");
            }
            else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Server"))
            {
                server = getStringValueFrom(doc, lCur, (const xmlChar*)"Address");
                serverPort = getStringValueFrom(doc, lCur, (const xmlChar*)"Port");
            }

            lCur = lCur->next;
        }


        // Create a new client for the specified server
        checkString(client, string("127.0.0.1"));
        checkInt(clientPort, 9000);
        checkString(server, string("127.0.0.1"));
        checkString(serverPort, string("9002"));

        address.reset(new OscClient(lo_address_new(server.c_str(), serverPort.c_str())));

        // We know the client port, we can launch our own server to receive messages from blobserver
        int configuratorPort = clientPort;
        // We want to use an unused port for this
        while (mOscServer == NULL)
        {
            configuratorPort += 10;
            char buffer[8];
            sprintf(buffer, "%i", configuratorPort);
            mOscServer = lo_server_thread_new(buffer, Configurator::oscError);
        }

        lo_server_thread_add_method(mOscServer, "/blobserver/connect", NULL, Configurator::oscHandlerConnect, this);
        lo_server_thread_add_method(mOscServer, NULL, NULL, Configurator::oscGenericHandler, this);
        lo_server_thread_start(mOscServer);
        mReady = true;

        // We need to sign in to the server before anything
        lo_send(address->get(), "/blobserver/signIn", "si", client.c_str(), configuratorPort);

        // Create and send the message to create this flow
        {
            atom::Message message;
            message.push_back(atom::StringValue::create(client.c_str()));
            //message.push_back(atom::IntValue::create(configuratorPort));
            message.push_back(atom::StringValue::create(detector.c_str()));
            for (int i = 0; i < sources.size(); ++i)
            {
                if (sources[i] == string(""))
                    continue;
                message.push_back(atom::StringValue::create(sources[i].c_str()));

                message.push_back(atom::IntValue::create(subsources[i]));
            }

            lo_message oscMessage = lo_message_new();
            atom::message_build_to_lo_message(message, oscMessage);
            lo_send_message(address->get(), "/blobserver/connect", oscMessage);
            lo_message_free(oscMessage);

            int index = 0;
            int waitLoop = 0;
            while (!index && waitLoop < LOAD_MAX_WAIT_TIME_MS)
            {
                nanosleep(&waitTime, NULL);
                waitLoop += (int)waitTime.tv_nsec / 1e6;

                index = mLastIndexReceived;
            }

            // If we received the message containing the index of the new flow
            if (index)
            {
                mLastIndexReceived = 0;

                simpleFlow.id = index;
                simpleFlow.address = client;
                simpleFlow.port = configuratorPort;
            }
            else
            {
                error = true;
            }
        }

        if (!error)
        {
            // The flow is created, we can now specify the parameters
            lCur = cur->xmlChildrenNode;
            int srcIndex = 0;

            while (lCur != NULL)
            {
                if (!xmlStrcmp(lCur->name, (const xmlChar*)"Detector"))
                {
                    if (lCur->xmlChildrenNode != NULL)
                    {
                        lCur = lCur->xmlChildrenNode;
                        while (true)
                        {
                            string paramName;
                            atom::Message values;
                            if (getParamValuesFrom(doc, lCur, paramName, values))
                            {
                                if (values.size() > 0)
                                {
                                    atom::Message message;
                                    message.push_back(atom::StringValue::create(simpleFlow.address.c_str()));
                                    message.push_back(atom::IntValue::create(simpleFlow.id));
                                    message.push_back(atom::StringValue::create((const char*)"Detector"));
                                    message.push_back(atom::StringValue::create(paramName.c_str()));
                                    for (int i = 0; i < values.size(); ++i)
                                        message.push_back(values[i]);

                                    lo_message oscMessage = lo_message_new();
                                    atom::message_build_to_lo_message(message, oscMessage);
                                    lo_send_message(address->get(), "/blobserver/setParameter", oscMessage);
                                    lo_message_free(oscMessage);
                                }
                            }

                            if (lCur->next == NULL)
                                break;
                            lCur = lCur->next;
                        }

                        lCur = lCur->parent;
                    }
                }
                else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Source"))
                {
                    if (lCur->xmlChildrenNode != NULL)
                    {
                        lCur = lCur->xmlChildrenNode;
                        while (true)
                        {
                            string paramName;
                            atom::Message values;
                            if (getParamValuesFrom(doc, lCur, paramName, values))
                            {
                                if (values.size() > 0)
                                {
                                    atom::Message message;
                                    message.push_back(atom::StringValue::create(simpleFlow.address.c_str()));
                                    message.push_back(atom::IntValue::create(simpleFlow.id));
                                    message.push_back(atom::StringValue::create((const char*)"Source"));
                                    message.push_back(atom::IntValue::create(srcIndex));
                                    message.push_back(atom::StringValue::create(paramName.c_str()));
                                    for (int i = 0; i < values.size(); ++i)
                                        message.push_back(values[i]);

                                    lo_message oscMessage = lo_message_new();
                                    atom::message_build_to_lo_message(message, oscMessage);
                                    lo_send_message(address->get(), "/blobserver/setParameter", oscMessage);
                                    lo_message_free(oscMessage);
                                }
                            }

                            if (lCur->next == NULL)
                                break;
                            lCur = lCur->next;
                        }

                        lCur = lCur->parent;
                    }

                    srcIndex++;
                }

                lCur = lCur->next;
            }
        }

        timespec nap;
        nap.tv_sec = 0;
        nap.tv_nsec = 1e8;
        nanosleep(&nap, NULL);

        {
            // We change the client port from the one used for configuration to the specified one
            lo_send(address->get(), "/blobserver/changePort", "si", client.c_str(), clientPort);

            // Now we can start the flow
            atom::Message message;
            message.push_back(atom::StringValue::create(simpleFlow.address.c_str()));
            message.push_back(atom::IntValue::create(simpleFlow.id));
            message.push_back(atom::StringValue::create((const char*)"Start"));

            lo_message oscMessage = lo_message_new();
            atom::message_build_to_lo_message(message, oscMessage);
            lo_send_message(address->get(), "/blobserver/setParameter", oscMessage);
            lo_message_free(oscMessage);
        }
    }

    return error;
}

/*************/
string Configurator::getStringValueFrom(const xmlDocPtr doc, const xmlNodePtr cur, const xmlChar* attr)
{
    string value = string("");

    if (cur->xmlChildrenNode != NULL)
    {
        xmlNodePtr lCur = cur->xmlChildrenNode;
        while (true)
        {
            if (!xmlStrcmp(lCur->name, attr))
            {
                xmlChar* key = xmlNodeListGetString(doc, lCur->xmlChildrenNode, 1);
                value = string((char*)key);
            }

            if (lCur->next == NULL)
                break;
            lCur = lCur->next;
        }
    }

    return value;
}

/*************/
int Configurator::getIntValueFrom(const xmlDocPtr doc, const xmlNodePtr cur, const xmlChar* attr)
{
    int value = 0;

    if (cur->xmlChildrenNode != NULL)
    {
        xmlNodePtr lCur = cur->xmlChildrenNode;
        while (true)
        {
            if (!xmlStrcmp(lCur->name, attr))
            {
                xmlChar* key = xmlNodeListGetString(doc, lCur->xmlChildrenNode, 1);
                stringstream str(string((char*)key));
                str >> value;
            }

            if (lCur->next == NULL)
                break;
            lCur = lCur->next;
        }
    }

    return value;
}

/*************/
bool Configurator::getParamValuesFrom(const xmlDocPtr doc, const xmlNodePtr cur, string& paramName, atom::Message& values)
{
    xmlNodePtr lCur = cur;

    if (!xmlStrcmp(lCur->name, (const xmlChar*)"Param"))
    {
        if (lCur->xmlChildrenNode != NULL)
        {
            lCur = lCur->xmlChildrenNode;
            lCur = lCur->next;
            if (!xmlStrcmp(lCur->name, (const xmlChar*)"Name"))
            {
                xmlChar* key = xmlNodeListGetString(doc, lCur->xmlChildrenNode, 1);
                paramName = string((char*)key);
    
                while (true)
                {
                    if (!xmlStrcmp(lCur->name, (const xmlChar*)"Value"))
                    {
                        xmlChar* key = xmlNodeListGetString(doc, lCur->xmlChildrenNode, 1);
                        stringstream str(string((char*)key));
                        float tmpValue;
                        str >> tmpValue;
                        if (!str)
                        {
                            values.push_back(atom::StringValue::create((char*)key));
                        }
                        else
                        {
                            values.push_back(atom::FloatValue::create(tmpValue));
                        }
                    }
                    
                    if (lCur->next == NULL)
                        break;
                    lCur = lCur->next;
                }
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

/*************/
void Configurator::checkString(string& str, const string defaultStr)
{
    if (str == string(""))
        str = defaultStr;
}

/*************/
void Configurator::checkInt(int& value, const int defaultValue)
{
    if (value == 0)
        value = defaultValue;
}

/*************/
void Configurator::oscError(int num, const char* msg, const char* path)
{
    std::cout << "liblo server error " << num << endl;
}

/*************/
int Configurator::oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    Configurator* object = (Configurator*)user_data;

    if(object->mVerbose)
    {
        std::cout << "Unhandled message received:" << std::endl;

        for(int i = 0; i < argc; ++i)
        {
            lo_arg_pp((lo_type)(types[i]), argv[i]);
        }

        std::cout << std::endl;
    }

    return 1;
}

/*************/
int Configurator::oscHandlerConnect(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    Configurator* object = static_cast<Configurator*>(user_data);

    atom::Message message;
    atom::message_build_from_lo_args(message, types, argv, argc);

    if (message.size() < 2)
    {
        cout << "Connect result message wrongly formated." << endl;
        return 1;
    }
    
    if (string(&types[1]) == string("s"))
    {
        // Error detected when trying to create flow
        string error = atom::toString(message[1]);
        cout << error << endl;
        return 1;
    }

    int index = 0;
    try
    {
        index = atom::toInt(message[1]);
    }
    catch (...)
    {
        return 1;
    }

    object->mLastIndexReceived.store(index);
}
