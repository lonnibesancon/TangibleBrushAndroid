package fr.limsi.ARViewer;

import android.os.AsyncTask;
import android.util.Log;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;


public class Client extends AsyncTask<String, String, String>{


	//protected String hostName = "192.168.1.41" ;       //Home computer
    //protected String hostName = "192.168.0.133" ;        //Aviz computer
    //protected String hostName = "192.168.1.101" ;        //Aviz computer2
//    protected String hostName="10.0.0.1";               //Local
	protected String hostName = "192.168.1.95";           //Mickael Computer
//	protected String hostName = "192.168.1.48";           //Mickael Computer
//	protected String hostName = "192.168.1.53";           //Mickael Computer
//	protected String hostName = "192.168.43.192";
    protected int portNumber = 8500;
    //protected Socket clientSocket ;
    protected DatagramSocket clientSocket ;
    protected InetAddress serverAddr;

    protected boolean connected = false;
    protected boolean closeConnection = false ;
    protected boolean valuesupdated = false ;
    protected boolean firstConnection = true ;
	protected boolean selectUpdated = false;
	protected boolean treatmentUpdated = false;
	protected boolean subDataUpdated = false;
	protected boolean modeUpdated = false;
	protected boolean pointSelectionDataUpdated = false;
	protected boolean tabletMatrixUpdated = false;
	protected boolean pIdUpdated = false;
	protected boolean tangoUpdated = false;

    protected short tangoEnable = 0 ;
    protected short considerX = 1 ;
    protected short considerY = 1 ;
    protected short considerZ = 1 ;

    protected String dataMatrix = "1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1;";
    protected String sliceMatrix = "1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1;";
    protected String seedPoint = "-1000000.0;-1000000.0;-1000000.0";
                                    //Dataset+showVolume+showSurface+showStylus+showSlice+showOutline+Matrix
    protected String dataToSend = "1;1;1;1;1;1;1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1;1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1;-1;-1;-1;" ;
	protected String selectData;
	protected String postTreatment;
	protected String subData;
	protected String pointSelectionData;
    protected String msg ;
	protected String mode;
	protected String tabletMatrixData;
	protected String tangoData;
	protected int pId;
    protected int dataset = 1 ;

    private long mLastTimestamp = 0;
    private long currentTimestamp = 1000000 ;
    private long refresh = 1;
    protected int counterTries = 0 ;

    protected boolean initDone = false ;

    

    public Client(){
        super();
        Log.e("Client created", "Client Created");
    }

    public Client(String hostName){
    	this.hostName = hostName ;
    }

    @Override
    protected void onPreExecute() {
        super.onPreExecute();
        if (connected == false) {
            try {
                serverAddr = InetAddress.getByName(hostName);
                Log.d("ClientActivity", "C: Connecting...");
                //clientSocket = new Socket(serverAddr, portNumber);
                clientSocket = new DatagramSocket();
                connected = true;
                Log.d("Connection status", "CONNECTED");
            } catch (Exception e) {
                Log.e("Client Application", "Error Opening"+e.getMessage(), e);
            }
        }
    }

    @Override
    protected void onPostExecute(String result){
        clientSocket.close();
        connected = false;
        Log.d("Socket State","Closed");
        super.onPostExecute(result);
    }

    @Override
    protected String doInBackground(String... f_url) {
    	if (connected == false) {
            try {
                serverAddr = InetAddress.getByName(hostName);
                Log.d("ClientActivity", "C: Connecting...");
                //clientSocket = new Socket(serverAddr, portNumber);
                clientSocket = new DatagramSocket();
                connected = true;
                Log.d("Connection status", "CONNECTED");
                String sentence = "coucou" ;
                byte[] sendData = sentence.getBytes();       
                DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, serverAddr, portNumber);       
                //clientSocket.send(sendPacket);
            } catch (Exception e) {
                Log.e("Client Application", "Error Opening"+e.getMessage(), e);
            }
        }

        while(closeConnection == false) {
            currentTimestamp = System.currentTimeMillis() ;
            long diff = currentTimestamp - mLastTimestamp ;

            //Log.d("Connected == ", "Connected = "+initDone);
            if (connected == true && initDone == true && (valuesupdated == true || selectUpdated == true || treatmentUpdated || modeUpdated || subDataUpdated) || pointSelectionDataUpdated){
            	//msg = ""+MATRIXCHANGED+";"+interactionMode+";"+mapperSelected+";"+matrix+PositionAndOrientation+this.seedPoint ;
				if(valuesupdated || firstConnection)
				{
					msg = "0;"+dataset+";"+dataToSend+considerX+";"+considerY+";"+considerZ+";";
					byte[] data = msg.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					//Log.d("Diff", "Diff = "+diff);
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);

					this.valuesupdated = false ;
					firstConnection = false ;
				}

				//Send the data for the selection
				if(selectUpdated)
				{
					byte[] data = selectData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					selectUpdated = false;
				}

				if(pointSelectionDataUpdated)
				{
					byte[] data = pointSelectionData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					pointSelectionDataUpdated = false;
				}

				if(treatmentUpdated)
				{
					byte[] data = postTreatment.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					treatmentUpdated = false;
				}

				if(subDataUpdated)
				{
					byte[] data = subData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					subDataUpdated = false;
				}

				if(modeUpdated)
				{
					byte[] data = mode.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					modeUpdated = false;
				}

				if(tabletMatrixUpdated)
				{
					byte[] data = tabletMatrixData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					modeUpdated = false;
				}

				if(tabletMatrixUpdated)
				{
					byte[] data = tabletMatrixData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					modeUpdated = false;
				}

				if(tangoUpdated)
				{
					byte[] data = tangoData.getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					tangoUpdated = false;
				}

				if(pIdUpdated)
				{
					byte[] data = ("9;"+Integer.toString(pId)).getBytes();
					DatagramPacket dp = new DatagramPacket(data, data.length, this.serverAddr, portNumber);
					counterTries = 0 ;
					do {
						try {
							clientSocket.send(dp);
							Log.d("MessageSent", ""+msg);
							break ;
						}catch (Exception e) {
							Log.e("ClientActivity", "SENDING ERROR "+ counterTries, e);
							counterTries ++ ;
						}
					}while(counterTries < 4);
					pIdUpdated = false;
				}
				mLastTimestamp = currentTimestamp ;
            }
        }

        if(closeConnection==true){
            connected = false ;
            clientSocket.close();
        }

        return "";
    }

    protected void setData(String s){
    	if(s.equals(this.dataToSend) == false){
    		this.dataToSend = s ;
	    	//Log.d("DataToSend", ""+dataToSend);
	    	this.valuesupdated = true ;
	        initDone = true ;	
    	}
    }

    protected void setDataMatrixString(String s){
        this.dataMatrix = s ;
        this.valuesupdated = true ;
        initDone = true ;
    }

	protected void setSelectionData(String s){
		this.selectData = s;
		this.selectUpdated = true;
	}

	protected void setPostTreatment(String s)
	{
		this.postTreatment = s;
		this.treatmentUpdated = true;
	}

    protected void setSliceMatrixString(String s){
    	this.sliceMatrix = s ;
    	this.valuesupdated = true ;
    	initDone = true ;
    }

	protected void setSubData(String s)
	{
		this.subData = s;
		this.subDataUpdated = true;
	}

	protected void setPointSelectionData(String s)
	{
		this.pointSelectionData=s;
		this.pointSelectionDataUpdated=true;
	}

    protected void setSeedPoint(String s){
        this.seedPoint = s ;
        this.valuesupdated = true ;
        initDone = true ;
 	}

	protected void setTabletMatrixData(String s)
	{
		this.tabletMatrixData = s;
		this.tabletMatrixUpdated = true;
	}

	protected void setInteractionMode(String s)
	{
		this.mode = s;
		this.modeUpdated = true;
	}

	protected void setPId(int p)
	{
		this.pId = p;
		this.pIdUpdated = true;
	}

	protected void setTangoData(String d)
	{
		this.tangoData = d;
		this.tangoUpdated = true;
	}
}
