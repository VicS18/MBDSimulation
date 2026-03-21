/*******************************************************************************
 * @author  Jiahao ZHANG
 * @email   jiahao.zhang96@gmail.com
 * @date    09/11/2022
 * @version 2.0
 *
 * TAM (Trusted Autonomous Mobility)
 * Copyright (c) 2013, 2021 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#include "F2MD_CPFacility.h"

using namespace omnetpp;
using namespace vanetza;
using namespace traci;

/**
   *  @brief  Creates a CPM Processing Facility module, with the given parameters
*/
F2MD_CPFacility::F2MD_CPFacility(const artery::VehicleDataProvider* VehProvider, const traci::VehicleController* VehController, F2MD_CPParameters& CPparams, unsigned long* PseudonymID){
    mVehicleDataProvider = VehProvider;
    mVehicleController = VehController;
    params = &CPparams;
    myPseudonym = PseudonymID;

}

/**
   *  @brief  Initialize the CPM Processing Facility module, with the given parameters
*/
void F2MD_CPFacility::initialize()
{
    params = params;
    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    // ------ Initialize F2MD  ------
    params->MAX_PLAUSIBLE_SPEED = vehicle_api.getMaxSpeed(id) + 0.1;
    params->MAX_PLAUSIBLE_ACCEL = vehicle_api.getAccel(id) + 0.1;
    params->MAX_PLAUSIBLE_DECEL = vehicle_api.getDecel(id) + 0.1;
    
    // ------ Initialize detectors ----
    mCpmChecks = CpmChecks(params);

    // Initialize globle parameters
    memset(accusedNodesCP,0,sizeof(accusedNodesCP));
    accusedNodesLengthCP = 0;


    // Depending on the Attacking Probability, the node will be an attacker or a genuine
    myMdType = induceMisbehavior(params->CP_LOCAL_ATTACKER_PROB);
    

    // ------ Initialize Attack  ------
    switch (myMdType) {
        // Genuine vehicle: green color
        case mbTypes::Genuine: {
            vehicle_api.setColor(id, libsumo::TraCIColor(0, 255, 0, 255));
            myAttackType = cpAttackTypes::Genuine;
        } break;

        case mbTypes::LocalAttacker: {
            if (params->MixCpLocalAttacks) {
                int AtLiSize = sizeof(params->MixCpLocalAttacksList) / sizeof(params->MixCpLocalAttacksList[0]);
                int localAttackIndex = 0;
                // each attacker executes a random attack
                if (params->RandomCpLocalMix) {
                    localAttackIndex = genLib.RandomInt(0, AtLiSize - 1);
                }
                else {
                    // mix all implemented attacks in order.
                    if (LastCPLocalAttackIndex < (AtLiSize - 1)) {
                        localAttackIndex = LastCPLocalAttackIndex + 1;
                        LastCPLocalAttackIndex = localAttackIndex;
                    }else {
                        localAttackIndex = 0;
                        LastCPLocalAttackIndex = 0;
                    }
                }
                myAttackType = params->MixCpLocalAttacksList[localAttackIndex];
            }
            else {
                myAttackType = params->CP_LOCAL_ATTACK_TYPE;
            }
            // Initialise attack
            mdAttack = MDCpmAttack();
            mdAttack.init(params, mVehicleController);

            // (Attacker) vehicle: red color
            vehicle_api.setColor(id, libsumo::TraCIColor(255, 0, 0, 255));
        } break;
        
        default:
            vehicle_api.setColor(id, libsumo::TraCIColor(0, 0, 0, 255));
            break;
    }

    detectedNodes = NodeTableCpm();


    int myId = mVehicleDataProvider->getStationId();
    string str_id = to_string(myId);



    // For Json
    string suffix = ".json";
    
    //if (myMdType == mbTypes::Genuine)
   //{ 
        //string outjsonfilecpmReport_name = "output/ATTACK/CPM/Report/logjson_cpm_report";
        string outjsonfilecpmReport_name = "output/reports/json/logjson_cpm_report_";
        outjsonfilecpmReport_name.append(str_id);
        //outjsonfilecpmReport_name.append (to_string(params->CP_LOCAL_ATTACK_TYPE));
        outjsonfilecpmReport_name.append (suffix);
        system("mkdir -p output/reports/json");
        outputjsonfilecpmReport.open(outjsonfilecpmReport_name,ios::out);
        jwritecpmReport.openJsonElementList("CPM_REPORT");
   //} 
}


/**
   *  @brief depending on node's attacking type (genuine or attacker), an attack is performed on the cpm before being sent
   *  @param CpmPayload_t* cpm payload data
*/
void F2MD_CPFacility::induceAttack(CpmPayload_t* cpm){
    if (myMdType == mbTypes::LocalAttacker && simTime().dbl() > params->START_CPM_ATTACK){
        for(int cpmContainerIndx = 0;  cpmContainerIndx < cpm->cpmContainers.list.count; cpmContainerIndx++){
            ConstraintWrappedCpmContainers_t* pd_DF = &cpm->cpmContainers;
            if(pd_DF->list.array[cpmContainerIndx]->containerId == CpmContainerId_perceivedObjectContainer){
                mdAttack.launchAttack(myAttackType, cpm);
            }
        }
        // std::cout << "---------------launch attack, success --------- \n" <<std::endl;
        
    }else {
        if (myMdType == mbTypes::Genuine) {
            // EV_INFO << "Genuine, nothing is done" << endl;
        }
        else {
            // EV_INFO << "AttackType : " << myAttackType << " with " << poc->list.count << "perceived object, nothing is done." << endl;
        }
    }

}





/**
   *  @brief defines the attack type of the node (genuine or attacker)
   *  @param double number of attacker already in the sim
*/
static double totalCPGenuine = 0;
static double totalCPLocalAttacker = 1; // 0
mbTypes::Mbs F2MD_CPFacility::induceMisbehavior(double LocalAttackerRatio)
{   
    // corrected, because the first node is always Genuinem, cant adapt our scenario
    double realFactor = totalCPLocalAttacker / (totalCPGenuine + totalCPLocalAttacker);
    if (LocalAttackerRatio > realFactor) {
        totalCPLocalAttacker++;
        return mbTypes::LocalAttacker;
    }
    else {
        totalCPGenuine++;
        return mbTypes::Genuine;
    }
}

/**
   *  @brief perform all the checks on the cpm received to detect a potential attack
   *  @param vanetza::asn1::Cpm& cpm received
*/
double F2MD_CPFacility::onCPM(const vanetza::asn1::Cpm& msg, const::vanetza::MacAddress& neighborMAcId,
                                         CpmCheckList& mCpmCheckList, std::map<int,int>& PseudoID2ArteryID){  
    
    // add local perception list
    vanetza::asn1::Cpm lastCpm;
    treatAttackFlags();

    // run checkst
    auto vehicle_api = mVehicleController->getTraCI();
    double checkfailed = mCpmChecks.CpmChecksum(msg, &detectedNodes, mCpmCheckList, &vehicle_api);

// ------------------------------------------------------------------------------------------------------

    int senderID = msg->header.stationId;

    // add the cpm received to the HistoryNodeTable
    if (!detectedNodes.includes(senderID)) {
        NodeHistoryCpm newNode(senderID);
        newNode.addCPM(msg);
        detectedNodes.put(senderID, newNode, params->MAX_CPM_HISTORY_TIME);
    }
    else {
        NodeHistoryCpm* existingNode = detectedNodes.getNodeHistoryAddr(
            senderID);

        lastCpm = *(existingNode->getLatestCPMAddr());
        existingNode->addCPM(msg);
    }

// ----------------------------------------------------------------------------------------------------
    if (checkfailed > 0) {
        addAccusedNode(senderID);
    }
    

// begin write the report copied from CAM

// Generate reports for the Misbehavior Authority
    if (checkfailed && (myMdType == mbTypes::Genuine)) {
        // std::cout << "********check report ******"<< std::endl;
        jwritecpmReport.addTagToElement("CPM_REPORT", writeReport(msg, neighborMAcId, mCpmCheckList, lastCpm, PseudoID2ArteryID));

       

        //makedecision (idsuspect, attacktype, location )

        //MDReport reportBase;
        //reportBase.setGenerationTime(simTime().dbl());
        //reportBase.setSenderPseudo(myPseudonym);
        //reportBase.setReportedPseudo(senderPseudo);

        //reportBase.setSenderRealId(myId);
        //reportBase.setReportedRealId(senderID);

        /*reportBase.setMbType(mbTypes::mbNames[BSMrecvd->getSenderMbType()]);
        reportBase.setAttackType(attackTypes::AttackNames[BSMrecvd->getSenderAttackType()]);
        reportBase.setSenderGps(Coord(curGPSCoordinates.x, curGPSCoordinates.y));
        reportBase.setReportedGps(BSMrecvd->getSenderGpsCoordinates());

        char nameV1[32] = "mdaV1";
        mdStats.getReport(nameV1, reportBase);
        if (params->writeReportsV1 && myBsmNum > 0) {
            writeReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
        }

        if (params->writeListReportsV1 && myBsmNum > 0) {
            writeListReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
        }

        if (params->sendReportsV1 && myBsmNum > 0) {
            sendReport(reportBase, version, mdv, bsmCheckV1, BSMrecvd);
        }*/
    }


//end write report

    return checkfailed;
}



/**
   *  @brief turn the color of a node into black if detected
*/
void F2MD_CPFacility::treatAttackFlags(){
    // int myID = mVehicleDataProvider->getStationId();
    auto myID = (*myPseudonym);
    // cout << "isAccusedNode(myID) " << myID << " :: " << isAccusedNode(myID) << endl;
    if (isAccusedNode(myID)){
        auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
        const std::string id = mVehicleController->getVehicleId();
        vehicle_api.setColor(id, libsumo::TraCIColor(255, 255, 255, 255));
    }
}

/**
   *  @brief adds the given ID to the list of detected nodes
*/
void F2MD_CPFacility::addAccusedNode(unsigned int id)
{
    bool found = false;
    for (int var = 0; var < accusedNodesLengthCP; ++var) {
        if (accusedNodesCP[var] == id) {
            found = true;
        }
    }

    if (!found) {
        accusedNodesCP[accusedNodesLengthCP] = id;
        accusedNodesLengthCP++;
    }
}


/**
   *  @result returns True is the node is already accused, False if not
*/
bool F2MD_CPFacility::isAccusedNode(unsigned int id)
{
    for (int var = 0; var < accusedNodesLengthCP; ++var) {
        if (accusedNodesCP[var] == id) {
            return true;
        }
    }
    return false;
}
/**
 * @brief print the final tag To json file
 * 
 */
std::string F2MD_CPFacility::printFinalTagtoJson() {
    std::string tempStr ="";
    JsonWriter jw;
    jw.openJsonElement("final", true);
    return jw.getJsonElement("final");
}


std::string F2MD_CPFacility::writeV2XPDU(const vanetza::asn1::Cpm& msg, const::vanetza::MacAddress& neighborMAcId, std::map<int,int>& PseudoID2ArteryID)
{
    string macid="";

    for (int i =0; i<neighborMAcId.octets.size();i++)
    {
        macid.append(std::to_string(neighborMAcId.octets[i]));
    } 

    JsonWriter jw;
    std::string tempStr ="";

    jw.openJsonElement("oneV2XPdu", true);

    tempStr = jw.getSimpleTag("sourcePseudoId", std::to_string(msg->header.stationId),true);
    jw.addTagToElement("oneV2XPdu", tempStr);

    if(params->EnablePC == true){
        tempStr = jw.getSimpleTag("sourceRealId", std::to_string(PseudoID2ArteryID.find(msg->header.stationId)->second), true);
        jw.addTagToElement("oneV2XPdu", tempStr);
    }else{
        tempStr = jw.getSimpleTag("sourceRealId", std::to_string(msg->header.stationId),true);
        jw.addTagToElement("oneV2XPdu", tempStr);
    }
    

    tempStr = jw.getSimpleTag("sourceMacId", macid,true);
    jw.addTagToElement("oneV2XPdu", tempStr);

    long rfT;
    const auto rftime = msg->payload.managementContainer.referenceTime;
    int res = asn_INTEGER2long(&rftime,&rfT);
    tempStr = jw.getSimpleTag("referenceTime",std::to_string(rfT),true);
    jw.addTagToElement("oneV2XPdu", tempStr);

    tempStr = jw.getSimpleTag("longitude", std::to_string(msg->payload.managementContainer.referencePosition.longitude),true);
    jw.addTagToElement("oneV2XPdu", tempStr);
    
    tempStr = jw.getSimpleTag("latitude", std::to_string(msg->payload.managementContainer.referencePosition.latitude),true);
    jw.addTagToElement("oneV2XPdu", tempStr);
    //std::cout << mVehicleDataProvider->getStationId() <<"   ******** msg->payload.cpmContainers.list.count ******" << msg->payload.cpmContainers.list.count << std::endl;
    
    for(int i=0; i< msg->payload.cpmContainers.list.count ; i++){
       if(msg->payload.cpmContainers.list.array[i]->containerId == CpmContainerId_originatingVehicleContainer){
            tempStr = jw.getSimpleTag("orientationAngle", std::to_string(msg->payload.cpmContainers.list.array[i]->containerData.choice.OriginatingVehicleContainer.orientationAngle.value),true);
            jw.addTagToElement("oneV2XPdu", tempStr);
       }

        if(msg->payload.cpmContainers.list.array[i]->containerId == CpmContainerId_perceivedObjectContainer){
            PerceivedObjectContainer_t* POC = &msg->payload.cpmContainers.list.array[i]->containerData.choice.PerceivedObjectContainer;
            jw.openJsonElement("PerceivedObjectContainer",false);
            tempStr = jw.getSimpleTag("numberOfPerceivedObjects", std::to_string(POC->numberOfPerceivedObjects),true);
            if(POC->numberOfPerceivedObjects > 0){
                jw.addTagToElement("PerceivedObjectContainer", tempStr);
                jw.openJsonElementList("objects");
                //cout << ":::::::::::" << POC->perceivedObjects.list.count << endl;
                for(int p=0; p < POC->perceivedObjects.list.count; p++){
                    jw.openJsonElement("object",true);
                    tempStr = jw.getSimpleTag("objectId", std::to_string(*POC->perceivedObjects.list.array[p]->objectId),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("measurementDeltaTime", std::to_string(POC->perceivedObjects.list.array[p]->measurementDeltaTime),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("position_xCoordinate", std::to_string(POC->perceivedObjects.list.array[p]->position.xCoordinate.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("position_yCoordinate", std::to_string(POC->perceivedObjects.list.array[p]->position.yCoordinate.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("velocityMagnitudeValue", std::to_string(POC->perceivedObjects.list.array[p]->velocity->choice.polarVelocity.velocityMagnitude.speedValue),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("velocityDirection", std::to_string(POC->perceivedObjects.list.array[p]->velocity->choice.polarVelocity.velocityDirection.value),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("accelerationMagnitudeValue", std::to_string(POC->perceivedObjects.list.array[p]->acceleration->choice.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue),true);
                    jw.addTagToElement("object", tempStr);
                    tempStr = jw.getSimpleTag("accelerationDirection", std::to_string(POC->perceivedObjects.list.array[p]->acceleration->choice.polarAcceleration.accelerationDirection.value),true);
                    jw.addTagToElement("object", tempStr);     
                    tempStr = jw.getSimpleTag("zAngularVelocity", std::to_string(POC->perceivedObjects.list.array[p]->zAngularVelocity->value),true);
                    jw.addTagToElement("object", tempStr);                                  
                    tempStr = jw.getSimpleTag("zAngle", std::to_string(POC->perceivedObjects.list.array[p]->angles->zAngle.value),true);
                    jw.addFinalTagToElement("object", tempStr);  
                    if (p < POC->perceivedObjects.list.count -1){ 
                        jw.addTagToElement("objects", jw.getJsonElement("object"));
                    }else{
                        jw.addFinalTagToElement("objects", jw.getJsonElement("object"));
                    } 
                }
                
                jw.addFinalTagToElement("PerceivedObjectContainer",jw.getJsonElementList("objects"));
            }else{
                jw.addFinalTagToElement("PerceivedObjectContainer", tempStr);
            }
            jw.addFinalTagToElement("oneV2XPdu",jw.getJsonElement("PerceivedObjectContainer"));  
       }

    }

    return jw.getJsonElement("oneV2XPdu");   

}


/**
   *  @brief write a new report
*/
std::string F2MD_CPFacility::writeReport(const vanetza::asn1::Cpm& msg, const::vanetza::MacAddress& neighborMAcId,
             CpmCheckList& mCpmCheckList, vanetza::asn1::Cpm& lastCpm, std::map<int,int>& PseudoID2ArteryID)
{
        //BasicCheckReport bcr = BasicCheckReport(reportBase);
        //bcr.setReportedCheck(bsmCheck);
        //bcr.writeStrToFile("../../testreport.json", params->serialNumber, "1",bcr.getReportPrintableJson(), NULL);

    //NodeHistoryCpm* NodeHistory = detectedNodes.getNodeHistoryAddr(msg->header.stationId);
    //vanetza::asn1::Cpm lastCpm = *(NodeHistory->getLatestCPMAddr());

    bool consistencycheckon = false;
    std::string tempStr ="";

    JsonWriter jw;
   
    jw.openJsonElement("report", true);

    tempStr = jw.getSimpleTag("generationTime",SIMTIME_STR(simTime()),true);
    jw.addTagToElement("report", tempStr);

    tempStr = jw.getSimpleTag("reporterPseudoId", std::to_string((*myPseudonym)),true);
    jw.addTagToElement("report", tempStr);

    tempStr = jw.getSimpleTag("reporterArteryId", std::to_string((mVehicleDataProvider->getStationId())),true);
    jw.addTagToElement("report", tempStr);

    tempStr = jw.getSimpleTag("version","1",true);
    jw.addTagToElement("report", tempStr);
    
    jw.openJsonElement("observationlocation", false);

    tempStr = jw.getSimpleTag("x", std::to_string(mVehicleDataProvider->position().x.value()),true);
    jw.addTagToElement("observationlocation", tempStr);

    tempStr = jw.getSimpleTag("y", std::to_string(mVehicleDataProvider->position().y.value()),true);
    jw.addFinalTagToElement("observationlocation", tempStr);


    jw.addTagToElement("report",jw.getJsonElement("observationlocation"));

    jw.openJsonElement("content",false);       

    jw.openJsonElementList("observationSet");
    for (int i=0; i< mCpmCheckList.getAttackedObject().size(); i++){
        jw.openJsonElement("observationByTarget", true);

        tempStr = jw.getSimpleTag("targetId", std::to_string(mCpmCheckList.getAttackedObject()[i].ObjectID),true);
        jw.addTagToElement("observationByTarget", tempStr);

        tempStr = jw.getSimpleTag("attackType", cpAttackTypes::AttackNames[params->CP_LOCAL_ATTACK_TYPE],false);
        jw.addTagToElement("observationByTarget", tempStr);

        tempStr = jw.getSimpleTag("check", std::to_string(mCpmCheckList.getAttackedObject()[i].Checkfailed),true);
        jw.addFinalTagToElement("observationByTarget", tempStr);

        if (mCpmCheckList.getAttackedObject()[i].Checkfailed > 3)
            consistencycheckon = true ; 

        if (i==mCpmCheckList.getAttackedObject().size()-1){
            jw.addFinalTagToElement("observationSet",jw.getJsonElement("observationByTarget"));

        }
        else{
            jw.addTagToElement("observationSet",jw.getJsonElement("observationByTarget"));

        } 
    } 

    jw.addTagToElement("content",jw.getJsonElementList("observationSet"));

    jw.openJsonElementList("V2XPduEvidence");
    
    
    //write current received CPM as evidence
    //std::cout << "****On est CURRENT CPM******" << std::endl;

    if (consistencycheckon)
    {
        jw.addTagToElement("V2XPduEvidence",writeV2XPDU(msg, neighborMAcId, PseudoID2ArteryID));
        jw.addFinalTagToElement("V2XPduEvidence",writeV2XPDU(lastCpm, neighborMAcId, PseudoID2ArteryID));
    } 
    else 
    {
        jw.addFinalTagToElement("V2XPduEvidence",writeV2XPDU(msg,neighborMAcId, PseudoID2ArteryID));
    } 
        
    //
    //std::cout << "****On est LAST CPM******" << std::endl;

    //write last received CPM as evidence 
   


    jw.addFinalTagToElement("content",jw.getJsonElementList("V2XPduEvidence"));  

    jw.addFinalTagToElement("report",jw.getJsonElement("content"));     
    return jw.getJsonElement("report");    
}

void F2MD_CPFacility::finish()
{
    jwritecpmReport.addFinalTagToElement("CPM_REPORT",printFinalTagtoJson());
    jwritecpmReport.writeHeader();
    jwritecpmReport.addElement(jwritecpmReport.getJsonElementList("CPM_REPORT"));  
    jwritecpmReport.writeFooter();
    if (outputjsonfilecpmReport.is_open()){
       
        outputjsonfilecpmReport << jwritecpmReport.getOutString()<<std::endl; 
    } 
    outputjsonfilecpmReport.close();
  

    //cSimpleModule::finish();
} 
