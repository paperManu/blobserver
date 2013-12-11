#include "configurator.h"

#include <cstring>
#include <sstream>
#include <atom/osc.h>
#include <glib.h>

using namespace std;

/*************/
Configurator::Configurator(int proto):
    mReady(false),
    mVerbose(false),
    mOscServer(NULL),
    mProto(proto)
{
    mLastIndexReceived = 0;
}


/*************/
Configurator::~Configurator()
{

}

/*************/
void Configurator::loadXML(const char* filename, bool distant)
{
    g_log(NULL, G_LOG_LEVEL_INFO, "%s::%s - Attempting to read XML file %s", __FILE__, __FUNCTION__, filename);

    xmlDocPtr doc;
    doc = xmlReadFile(filename, NULL, 0);

    if (doc == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - Failed to parse %s", __FILE__, __FUNCTION__, filename);
        return;
    }

    xmlNodePtr cur;
    cur = xmlDocGetRootElement(doc);
    if (cur == NULL || cur->xmlChildrenNode == NULL)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - Configuration file seems to be empty", __FILE__, __FUNCTION__);
        return;
    }

    cur = cur->xmlChildrenNode;

    while (cur != NULL)
    {
        if (!xmlStrcmp(cur->name, (const xmlChar*)"Flow"))
        {
            bool error = loadFlow(doc, cur, distant);
            if (error)
            {
                g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - An error has been detected while parsing file %s", __FILE__, __FUNCTION__, filename);
            }
        }

        cur = cur->next;
    }
}

/*************/
bool Configurator::loadFlow(const xmlDocPtr doc, const xmlNodePtr cur, bool distant)
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
    lo_server_thread oscServer = NULL;

    string nullString = string("");

    if (cur->xmlChildrenNode == NULL)
        error = true;
    else
    {
        lCur = cur->xmlChildrenNode;

        // We need to get the actuator name, as well as information about sources
        // to create the flow (before any change of parameters)
        string actuator;
        vector<string> sources;
        vector<string> subsources;
        string client, realClient, server;
        string serverPort;
        int clientPort = 0;

        while (lCur != NULL)
        {
            if (!xmlStrcmp(lCur->name, (const xmlChar*)"Actuator"))
            {
                actuator = getStringValueFrom(doc, lCur, (const xmlChar*)"Type");
            }
            else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Source"))
            {
                sources.push_back(getStringValueFrom(doc, lCur, (const xmlChar*)"Type"));
                subsources.push_back(getStringValueFrom(doc, lCur, (const xmlChar*)"Subsource"));
            }
            else if (!xmlStrcmp(lCur->name, (const xmlChar*)"Client"))
            {
                realClient = getStringValueFrom(doc, lCur, (const xmlChar*)"Address");
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
        checkString(realClient, string("127.0.0.1"));
        if (distant)
            client = realClient;
        else
            client = string("127.0.0.1");
        checkInt(clientPort, 9000);
        checkString(server, string("127.0.0.1"));
        checkString(serverPort, string("9002"));

        address.reset(new OscClient(lo_address_new_with_proto(mProto, server.c_str(), serverPort.c_str())));

        // We know the client port, we can launch our own server to receive messages from blobserver
        int configuratorPort = clientPort;
        // We want to use an unused port for this
        while (oscServer == NULL)
        {
            configuratorPort += 10;
            char buffer[8];
            sprintf(buffer, "%i", configuratorPort);
            oscServer = lo_server_thread_new_with_proto(buffer, mProto, Configurator::oscError);
        }

        lo_server_thread_add_method(oscServer, "/blobserver/connect", NULL, Configurator::oscHandlerConnect, this);
        lo_server_thread_add_method(oscServer, NULL, NULL, Configurator::oscGenericHandler, this);
        lo_server_thread_start(oscServer);
        mReady = true;

        // We need to sign in to the server before anything
        lo_send(address->get(), "/blobserver/signIn", "si", client.c_str(), configuratorPort);
        // We send a message to set client port to configuratorPort, in case we were already signed in
        lo_send(address->get(), "/blobserver/changePort", "si", client.c_str(), configuratorPort);

        // Create and send the message to create this flow
        {
            atom::Message message;
            message.push_back(atom::StringValue::create(client.c_str()));
            message.push_back(atom::StringValue::create(actuator.c_str()));
            for (int i = 0; i < sources.size(); ++i)
            {
                if (sources[i] == string(""))
                    continue;
                message.push_back(atom::StringValue::create(sources[i].c_str()));

                message.push_back(atom::StringValue::create(subsources[i].c_str()));
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
                if (!xmlStrcmp(lCur->name, (const xmlChar*)"Actuator"))
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
                                    message.push_back(atom::StringValue::create((const char*)"Actuator"));
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
            // We change the client ip and port from the one used for configuration to the specified one
            lo_send(address->get(), "/blobserver/changeIp", "ssi", "127.0.0.1", realClient.c_str(), clientPort);

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

        if (oscServer != NULL)
        {
            lo_server_thread_stop(oscServer);
            lo_server_thread_free(oscServer);
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
                        char* token = strtok((char*)key, " ");

                        int index = 0;
                        while (str)
                        {
                            float tmpValue;
                            string tmpString;
                            str >> tmpValue;
                            if (str)
                            {
                                values.push_back(atom::FloatValue::create(tmpValue));
                                index ++;
                            }
                            else
                            {
                                for (int i = 0; i < index; ++i)
                                    token = strtok(NULL, " ");

                                if (token == NULL)
                                    continue;

                                values.push_back(atom::StringValue::create(token));
                            }
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
    g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - liblo server error %i", __FILE__, __FUNCTION__, num);
}

/*************/
int Configurator::oscGenericHandler(const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data)
{
    Configurator* object = (Configurator*)user_data;

    if(object->mVerbose)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - Unhandled message received", __FILE__, __FUNCTION__);

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

    if (message.size() < 1)
    {
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - Error detected in the connect result: message is too short to be well formed", __FILE__, __FUNCTION__);
        return 1;
    }
    
    if (string(&types[0]) == string("s"))
    {
        // Error detected when trying to create flow
        string error = atom::toString(message[0]);
        g_log(NULL, G_LOG_LEVEL_WARNING, "%s::%s - %s", __FILE__, __FUNCTION__, error.c_str());
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
