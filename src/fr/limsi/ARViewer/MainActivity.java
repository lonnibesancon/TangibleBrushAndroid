package fr.limsi.ARViewer;

import java.io.PrintWriter;

import android.app.AlertDialog;
import android.content.res.Resources;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.ColorDrawable;
import android.hardware.Camera;
import android.os.Bundle;
import android.util.Log;
import android.util.TypedValue;
import android.view.GestureDetector;
import android.view.GestureDetector.SimpleOnGestureListener;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.ToggleButton;
import android.widget.ImageButton;

import android.app.*;
import android.bluetooth.*;
import android.content.*;
import android.graphics.*;
import android.hardware.*;
import android.hardware.Camera;
import android.hardware.usb.*;
import android.os.*;
import android.util.*;
import android.view.*;
import android.widget.*;
import java.io.*;
import java.net.*;
import java.lang.*;
import java.util.*;
import android.view.View.OnClickListener;
import java.lang.Object;


import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
 
import java.util.ArrayList;
import java.util.List;

import java.util.UUID;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;


import java.text.DecimalFormat ;

import com.google.atap.tangoservice.*;



// import com.qualcomm.QCAR.QCAR;
// // import com.qualcomm.ar.pl.CameraPreview;
import com.lannbox.rfduinotest.RFduinoService;
import com.lannbox.rfduinotest.EditData;
import com.lannbox.rfduinotest.HexAsciiHelper;

public class MainActivity extends BaseARActivity
 implements View.OnTouchListener, GestureDetector.OnDoubleTapListener, Tango.OnTangoUpdateListener, SensorEventListener, InteractionMode, OnClickListener, BluetoothAdapter.LeScanCallback //, View.OnLongClickListener
    // , CameraPreview.SizeCallback
{
    private static final String TAG = Config.APP_TAG;
    private static final boolean DEBUG = Config.DEBUG;

    private ImageButton tangibleBtn ;
    //private Button sliceBtn ;
    private Button constrainXBtn ;
    private Button constrainYBtn ;
    private Button constrainZBtn ;
    private Button autoConstrainBtn ;
    private ToggleButton dataORplaneTangibleToggle ;
    private ToggleButton dataORplaneTouchToggle ;
    private ToggleButton tangibleToggle ;
    private ToggleButton touchToggle ;
    private ToggleButton selectionToggle ;
    private ToggleButton constrainToggle ;
    private Button resetBtn ;

    private boolean isTangibleOn = true ;
    private boolean isTouchOn = true ;
    private boolean dataORplaneTangible = true ; //Data
    private boolean dataORplaneTouch = true ;    //Data
    private boolean isTangiblePressed = false ;

    private Button seedingBtn ;
    //private Button translateBtn ;
    //private ToggleButton dataORplane ;
    //private ToggleButton translateBtn ;

    public Object lock = new Object() ;

    // FIXME: static?
    private static FluidMechanics.Settings fluidSettings = new FluidMechanics.Settings();
    private static FluidMechanics.State fluidState = new FluidMechanics.State();

    private FluidMechanics.Settings tempSettings = new FluidMechanics.Settings();

    // private static boolean mFluidDataLoaded = false;
    private static int mDataSet = head;
    private static boolean mDatasetLoaded = false;
    private static boolean mVelocityDatasetLoaded = false;

    // Menu            
    private static boolean menuInitialized = false;
    private MenuItem mAxisClippingMenuItem, mStylusClippingMenuItem;

    // Pinch zoom
    private float mInitialPinchDist = 0;
    private float mInitialZoomFactor = 0;
    private boolean mZoomGesture = false;

    // Gestures
    private GestureDetector mGestureDetector;

    private ARSurfaceView mView;

    private Tango mTango;
    private TangoConfig mConfig;
    private boolean mIsTangoServiceConnected;

    // Gyro part
    private SensorManager mSensorManager;
    private Sensor mGyro ;
    private Sensor mRotation ;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private long mLastTimestamp = 0;

    private Client client ;

    private int interactionMode = nothing;
    private boolean tangibleModeActivated = false ;

    //LOGGING
    private static final String FILENAME = "myFile.txt";
    private Logging logging ;
    private short interactionType ;
    private FileOutputStream fOut ;
    private OutputStreamWriter outputStreamWriter ;
    private boolean isInteracting = false ;
    private long initialTime ;
    private long previousLogTime = 0 ;
    private long logrefreshrate = 50 ;
    private int nbOfFingers = 0;
    private int nbOfFingersButton = 0 ;
    private int nbOfResets = 0 ;

    private final ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);;

    //Constrain interaction part 
    private boolean constrainX ;
    private boolean constrainY ;
    private boolean constrainZ ;
    private boolean constrainRotation ;
    private boolean constrainTranslation ;
    private boolean autoConstraint ;
    private boolean isConstrained = false ;
    private boolean dataOrTangibleValue = true ;
    private TextView bluetoothState ;


    public int pId = -1 ;
    private boolean fileOpened = false ;


    @Override
    protected int getAppType() {
        // return NativeApp.APP_TYPE_STUB;
        return NativeApp.APP_TYPE_FLUID;
    }


    private void getStartInfo(){
        AlertDialog.Builder alert = new AlertDialog.Builder(this);
        
        alert.setTitle("Participant ID");
        alert.setMessage("Please enter the ID");

        // Set an EditText view to get user input 
        final EditText input = new EditText(this);
        alert.setView(input);

        alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int whichButton) {
            if(input.getText()!= null){
                int p = Integer.parseInt(input.getText().toString());
                if(p < 0){
                    System.exit(0);
                }
                else{
                    pId = p ;
                    openFile();
                }
            }
            
          
          }
        });

        alert.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int whichButton) {
            System.exit(0);
          }
        });

        alert.show();
    }

    private void openFile(){
        try{
            fOut = new FileOutputStream("/sdcard/test"+"/"+pId+".csv");
            outputStreamWriter = new OutputStreamWriter(fOut);
        }
        catch (IOException e) {
            Log.e(TAG, "File write failed: " + e.toString());
        } 

        this.executor.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                try{
                    loggingFunction();
                }
                catch (IOException e) {
                    Log.e(TAG, "File write failed: " + e.toString());
                } 
            }
        }, 0L, logrefreshrate , TimeUnit.MILLISECONDS);

    }

    @Override
    @SuppressWarnings("deprecation")
    public void onCreate(Bundle savedInstanceState) {
        boolean wasInitialized = isInitialized();

        super.onCreate(savedInstanceState);

        if (!isInitialized()) // || !isCameraAvailable())
            return;

        getStartInfo();

        FluidMechanics.getSettings(fluidSettings);
        FluidMechanics.getState(fluidState);
        fluidSettings.precision = 1 ;
        fluidSettings.translatePlane = false ;
        interactionMode = planeTouchTangible ;
        isTangibleOn = true ;
        isTouchOn = true ;
        //fluidSettings.dataORplane = 0 ; //Data 

        this.client = new Client();
        this.client.execute();
        // Request an overlaid action bar (title bar)
        getWindow().requestFeature(Window.FEATURE_ACTION_BAR_OVERLAY);

        // setContentView(R.layout.main);
        setContentView(R.layout.main_noar);

        mView = (ARSurfaceView)findViewById(R.id.glSurfaceView);
        setView(mView);

        // ContextMenuFragment content = new ContextMenuFragment();
        // getFragmentManager().beginTransaction().add(android.R.id.content, content).commit();
        registerForContextMenu(mView);

        mView.setOnTouchListener(this);

        mGestureDetector = new GestureDetector(new SimpleOnGestureListener() {});
        //mGestureDetector.setOnDoubleTapListener(this);
        // view.setOnLongClickListener(this);

        setupActionBar();
        //setupSlider();
        setupSliderPrecision();

        createBluetooth();

        if (!wasInitialized) {
            mDataSet = 0;
            loadNewData();
        }

        // =================================================
        // Tango service

        // Instantiate Tango client
        mTango = new Tango(this);

        // Set up Tango configuration for motion tracking
        // If you want to use other APIs, add more appropriate to the config
        // like: mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true)
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        
        FluidMechanics.setInteractionMode(this.interactionMode);

        this.tangibleBtn = (ImageButton) findViewById(R.id.tangibleBtn);
        //this.tangibleBtn.setOnClickListener(this);
        this.tangibleBtn.setOnTouchListener(this);
        /*this.tangibleBtn.setOnTouchListener(new View.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                 if (event.getAction() == MotionEvent.ACTION_DOWN ){
                    isTangibleOn = true ;
                    FluidMechanics.buttonPressed();
                }
                else if(event.getAction() == MotionEvent.ACTION_UP ){
                    isTangibleOn = false ;
                    FluidMechanics.buttonReleased();
                }
                //int index = event.getActionIndex();
                //fingerOnButtonIndex = event.getPointerId(index);
             return true;
            }
        });*/

        //this.sliceBtn = (Button) findViewById(R.id.sliceBtn);
        //this.sliceBtn.setOnClickListener(this);
        //this.tangibleBtn.setOnTouchListener(this);

        /*this.constrainXBtn = (Button) findViewById(R.id.constrainX);
        //this.constrainXBtn.setOnClickListener(this);
        this.constrainXBtn.setOnTouchListener(this);

        this.constrainYBtn = (Button) findViewById(R.id.constrainY);
        //this.constrainYBtn.setOnClickListener(this);
        this.constrainYBtn.setOnTouchListener(this);

        this.constrainZBtn = (Button) findViewById(R.id.constrainZ);
        //this.constrainZBtn.setOnClickListener(this);
        this.constrainZBtn.setOnTouchListener(this);

        this.autoConstrainBtn = (Button) findViewById(R.id.autoConstrain);
        //this.autoConstrainBtn.setOnClickListener(this);
        this.autoConstrainBtn.setOnTouchListener(this);*/

        resetBtn = (Button) findViewById(R.id.resetBtn);
        this.resetBtn.setOnClickListener(this);

        this.seedingBtn = (Button) findViewById(R.id.seedingBtn);
        this.seedingBtn.setOnTouchListener(this);


        /*touchToggle = (ToggleButton) findViewById(R.id.touchToggle);
        touchToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                
                if (isChecked) {
                    isTouchOn = true ;
                    dataORplaneTouchToggle.setEnabled(true);
                } else {
                    isTouchOn = false ;
                    dataORplaneTouchToggle.setEnabled(false);
                }
                setInteractionMode();
            }

        });*/


        selectionToggle = (ToggleButton) findViewById(R.id.selectionToggle);
        selectionToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    interactionMode = dataTouchTangible ;
                } else {
                    interactionMode = planeTouchTangible ;
                }
                //setInteractionMode();
                FluidMechanics.setInteractionMode(interactionMode);
				client.setInteractionMode("6" + interactionMode);
				Log.e("Main", "setInteractionMode");
            }

        });

        constrainToggle = (ToggleButton) findViewById(R.id.constrainToggle);
        constrainToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
					Log.e("Main", "isChecked");
                    fluidSettings.constrainSelection = true ;
                } else {
                    fluidSettings.constrainSelection = false ;
                }
				FluidMechanics.setConstrainSelection(fluidSettings.constrainSelection);
            }
        });

        /*dataORplaneTouchToggle = (ToggleButton) findViewById(R.id.dataORplaneTouch);
        dataORplaneTouchToggle.setChecked(true);
        dataORplaneTouchToggle.setEnabled(false);
        dataORplaneTouchToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {    
                if (isChecked) {
                    dataORplaneTouch = true ;
                } else {
                    dataORplaneTouch = false ;
                }
                setInteractionMode();
            }

        });

        bluetoothState = (TextView) findViewById(R.id.textOverlay);
        /*this.tangibleBtn.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG,"Button Clicked");
                Toast.makeText(MainActivity.this, "Button Clicked", Toast.LENGTH_SHORT).show();
            }
        });*/

        Log.d(TAG,"Listener Set");

        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        int width = size.x;
        int height = size.y;

        Log.d(TAG,"width = "+width+" height = "+height);

        Date date = new Date();
        initialTime = date.getTime();
        logging = new Logging();

        // mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true); // default is true

        // mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);

        //executor = new ScheduledThreadPoolExecutor(1);

        KeyguardManager keyguardManager = (KeyguardManager) getSystemService(Activity.KEYGUARD_SERVICE);  
        KeyguardManager.KeyguardLock lock = keyguardManager.newKeyguardLock(KEYGUARD_SERVICE);  
        lock.disableKeyguard();  

        
    }

    //private void setInteractionMode(){
        /*if(isTangibleOn == false){
            if(isTouchOn == false){
                interactionMode = nothing ;
                //fluidSettings.interactionMode = nothing ;
                Log.d(TAG,"No Interaction");
            }
            else{
                if(dataORplaneTouch == false){
                    interactionMode = planeTouch ;
                    Log.d(TAG,"planeTouch");
                    //fluidSettings.interactionMode = planeTouch ;
                }
                else{
                    interactionMode = dataTouch ;
                    Log.d(TAG,"dataTouch");
                    //fluidSettings.interactionMode = dataTouch ;
                }
            }
        }

        else{
            if(isTouchOn == false){
                if(dataORplaneTangible == false){
                    interactionMode = planeTangible ;
                    Log.d(TAG,"planeTangible");
                    //fluidSettings.interactionMode = planeTangible ;
                }
                else{
                    interactionMode = dataTangible ;
                    Log.d(TAG,"dataTangible");
                    //fluidSettings.interactionMode = dataTangible ;
                }
            }
            else{*/
                /*if(dataORplaneTangible == false && dataORplaneTouch == false){
                    interactionMode = planeTouchTangible ;
                    Log.d(TAG,"planeTouchTangible");
                    //fluidSettings.interactionMode = planeTouchTangible ;
                }
                else if(dataORplaneTangible == true && dataORplaneTouch == true){
                    interactionMode = dataTouchTangible ;
                    Log.d(TAG,"dataTouchTangible");
                    //fluidSettings.interactionMode = dataTouchTangible ;
                }
                else if(dataORplaneTangible == false && dataORplaneTouch == true){
                    interactionMode = dataPlaneTouchTangible ;
                    Log.d(TAG,"dataPlaneTouchTangible");
                    //fluidSettings.interactionMode = dataPlaneTangibleTouch ;
                }
                else if(dataORplaneTangible == true  && dataORplaneTouch == false){
                    interactionMode = dataPlaneTangibleTouch ;
                    Log.d(TAG,"dataPlaneTangibleTouch");
                    //fluidSettings.interactionMode = dataPlaneTangibleTouch ;
                }*/
            //}
        //}

        //FluidMechanics.setInteractionMode(interactionMode);
    //}


    protected void loggingFunction()throws IOException{
        /*if(this.isInteracting == false){
            return ;
        }
        else{*/
            Date date = new Date();
            long current = date.getTime();
            long timestamp = current - initialTime ;
            synchronized(lock){
                writeLine(timestamp);
            }
            this.isInteracting = false ;
            //Log.d(TAG,"Logging");
        //}

        
        /*Date date = new Date();
        long current = date.getTime();

        long timestamp = current - initialTime ;
        //if(timestamp - previousLogTime > logrefreshrate){
            Log.d(TAG,"Timestamp = "+timestamp);
            synchronized(lock){
                logging.addLog(timestamp,fluidSettings.precision,(short)interactionMode,interactionType,phase,
                           isConstrained, constrainX, constrainY, constrainZ, autoConstraint);    
            }
            previousLogTime = timestamp ;
        //}*/
        
        

    }

    private void closeFile(){
        try{
            outputStreamWriter.close();
            fOut.close();
        }
        
        catch (IOException e) {
            Log.e(TAG, "Close failed" + e.toString());
        } 
    }

    /*private void writeInfo(){
        Log.d(TAG,"Writing Info to SD CARD to "+"/sdcard/test"+"/"+FILENAME);
        try {
            //OutputStreamWriter outputStreamWriter = new OutputStreamWriter(openFileOutput("/sdcard/test"+"/"+FILENAME, Context.MODE_PRIVATE));
            FileOutputStream fOut = new FileOutputStream("/sdcard/test"+"/"+FILENAME);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(fOut);
            
            for(int i = 0 ; i < logging.size ; i++){
                outputStreamWriter.write(logging.getString(i));    
            }
            outputStreamWriter.close();
            fOut.close();
        }
        catch (IOException e) {
            Log.e(TAG, "File write failed: " + e.toString());
        } 

    }*/

    private String getLogString(long timestamp){
        //Log.d(TAG,"LOGGING");
        return (""+timestamp+";"
                 +mDataSet+";"
                 +interactionMode+";"
                 +fluidSettings.precision+";"
                 +interactionType+";"
                 +nbOfFingers+";"
                 +isConstrained+";"
                 +constrainX+";"
                 +constrainY+";"
                 +constrainZ+";"
                 +autoConstraint+";"
                 +nbOfFingers+";"
                 +nbOfFingersButton+";"
                 +nbOfResets+";"
                 +"\n") ;
                //LOG NBFINGERS ON SCREEN
    }

    private void writeLine(long timestamp) throws IOException{
        outputStreamWriter.write(getLogString(timestamp));
        //outputStreamWriter.flush();
    }


    private void writeLogging(){
        Log.d(TAG,"Writing Info to SD CARD to "+"/sdcard/test"+"/"+FILENAME);
        try {
             //OutputStreamWriter outputStreamWriter = new OutputStreamWriter(openFileOutput("/sdcard/test"+"/"+FILENAME, Context.MODE_PRIVATE));
            FileOutputStream fOut = new FileOutputStream("/sdcard/test"+"/"+FILENAME);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(fOut);
            
            for(int i = 0 ; i < logging.size ; i++){
                outputStreamWriter.write(logging.getString(i));    
            }
            outputStreamWriter.close();
            fOut.close();
        }
        catch (IOException e) {
            Log.e(TAG, "File write failed: " + e.toString());
        } 
    }

    private void setTangoListeners() {
        // Select coordinate frame pairs
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                           TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE, // OK?
                           // TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION, // needs KEY_BOOLEAN_LEARNINGMODE == true
                           TangoPoseData.COORDINATE_FRAME_DEVICE
                           ));
        // Add a listener for Tango pose data
        mTango.connectListener(framePairs, this);
    }

    // private static final int SECS_TO_MILLISECS = 1000;
    // private static final double UPDATE_INTERVAL_MS = 50.0;
    // private double mPreviousTimeStamp;
    // private double mTimeToNextUpdate = UPDATE_INTERVAL_MS;
    @Override
    public void onPoseAvailable(TangoPoseData pose) {
        // Log.d(TAG, "onPoseAvailable");

        // if (pose.statusCode == TangoPoseData.POSE_INVALID) {
        if (pose.statusCode != TangoPoseData.POSE_VALID && mDatasetLoaded == true) {
            Log.w(TAG, "Invalid pose state");

            // if (mTranslationsEnabled) {
            //     runOnUiThread(new Runnable() {
            //         @Override
            //         public void run() {
            //             mTextOverlay.setVisibility(View.VISIBLE);
            //             mTextOverlay.setText("Position non détectée !");
            //         }});
            // }
        } else {
            //if(isTangibleOn){     //Can't use it because we still need to monitor the position of the tango
                this.interactionType = tangibleInteraction ;
                FluidMechanics.setTangoValues(pose.translation[0],pose.translation[1],pose.translation[2],
                                          pose.rotation[0],pose.rotation[1],pose.rotation[2],pose.rotation[3] ) ;
                
                //synchronized(lock){
                    
                //}
            //}
            
            // runOnUiThread(new Runnable() {
            //     @Override
            //     public void run() {
            //         mTextOverlay.setVisibility(View.INVISIBLE);
            //     }});
        }
        if(isTangiblePressed){
            this.isInteracting = true ;
            requestRender();
        }
        

    }

    @Override
    public void onXyzIjAvailable(TangoXyzIjData arg0) {
        // Ignoring XyzIj data
    }

    @Override
    public void onTangoEvent(TangoEvent arg0) {
        // Ignoring TangoEvents
    }

    @Override
    public void onFrameAvailable(int arg0) {
        // Ignoring onFrameAvailable Events

    }

    @Override
   public void onSensorChanged(SensorEvent event) {
       if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE && mDatasetLoaded == true ) {
           if (mLastTimestamp != 0) {
               float dt = (event.timestamp - mLastTimestamp) * NS2S;
               if (dt != 0 && isTangiblePressed) {
                   FluidMechanics.setGyroValues(dt * event.values[0],   // rx
                                                dt * event.values[1],   // ry
                                                dt * event.values[2],0);  // rz
                   this.interactionType = tangibleInteraction ;
                   requestRender();

                   
               }
           }
           mLastTimestamp = event.timestamp;
       } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
           // Attention aux axes qui peuvent varier selon le device !
           /*NativeApp.setAbsoluteOrientation(
               event.values[0], // x
               event.values[1], // y
               event.values[2], // z
               event.values[3]  // w
           );*/

            double x = Math.asin(event.values[0]);
            float y = event.values[1];
            float z = event.values[2];

            if(x <= (Math.PI/4 + Math.PI/8) && x >= (Math.PI/4 - Math.PI/8) ){
                //Log.d(TAG,"ZZZZZZZZZZZZZZZZZZZZZZ");
            }
            if(x >= (0 + Math.PI/8) && x <= (Math.PI/2 - Math.PI/8) ){
                //Log.d(TAG,"YYYYYYYYYYYYYYYYYYYYYY");
            }

            //Log.d(TAG,"Rotation X = "+event.values[0] );
            //Log.d(TAG,"Rotation Y = "+event.values[1] );
            //Log.d(TAG,"Rotation Z = "+event.values[2] );
            //Log.d(TAG,"Rotation X = "+x+" -- Rotation Y = "+y+" -- Rotation Z = "+z);
       } /*else if(event.sensor.getType() == Sensor.TYPE_ORIENTATION){
            Log.d(TAG,"Rotation X = "+event.values[0]+" -- Rotation Y = "+event.values[1]+" -- Rotation Z = "+event.values[2]);
       }*/
       
   }

   @Override
   public void onAccuracyChanged(Sensor sensor, int accuracy) {
       // Log.d(TAG, "onAccuracyChanged");
   }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
            mIsTangoServiceConnected = false;
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), "Tango Error!",
                           Toast.LENGTH_SHORT).show();
        }
        mSensorManager.unregisterListener(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsTangoServiceConnected) {
            try {
                setTangoListeners();
            } catch (TangoErrorException e) {
                Toast.makeText(this, "Tango Error! Restart the app!",
                               Toast.LENGTH_SHORT).show();
            }
            try {
                mTango.connect(mConfig);
                mIsTangoServiceConnected = true;
            } catch (TangoOutOfDateException e) {
                Toast.makeText(getApplicationContext(),
                               "Tango Service out of date!", Toast.LENGTH_SHORT)
                    .show();
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(),
                               "Tango Error! Restart the app!", Toast.LENGTH_SHORT)
                    .show();
            }
        }
        mSensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotation, SensorManager.SENSOR_DELAY_UI);
    }

    @Override
    public void onStop () {
        Log.d(TAG,"Finish Activity");
        bluetoothAdapter.stopLeScan(this);

        unregisterReceiver(scanModeReceiver);
        unregisterReceiver(bluetoothStateReceiver);
        unregisterReceiver(rfduinoReceiver);
        //writeLogging();
        super.onStop() ;
    }

    private void loadNewData() {
        loadDataset(++mDataSet);
    }

    private void loadDataset(int id) {
        mDatasetLoaded = false;

        mDataSet = (id % 4);
        client.dataset = mDataSet ;

        // TODO: check exceptions + display error message
        // TODO: load in background?
        switch (mDataSet) {
        // switch ((mDataSet++ % 3)) {
            case ftle:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("ftlelog.vtk", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = ftle ;
                break;
            case ironprot:
                // FluidMechanics.loadDataset(copyAssetsFileToStorage("head.vti", false));
                FluidMechanics.loadDataset(copyAssetsFileToStorage("ironProt.vtk", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = ironprot ;
                break;
            case head:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("head.vti", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = head ;
                break;
            case velocities:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("FTLE7.vtk", false));
                FluidMechanics.loadVelocityDataset(copyAssetsFileToStorage("Velocities7.vtk", false));
                mVelocityDatasetLoaded = true;
                //client.dataset = velocities ;
                break;

            /*case 4:
                //this.finishAffinity();
                //android.os.Process.killProcess(android.os.Process.myPid());
                //System.exit(0);
                break ;*/

        }

        // We want the large display to change as well:
        client.valuesupdated = true ; 


        // Apply the computed zoom factor
        updateDataState();
        // settings.zoomFactor = fluidState.computedZoomFactor;
        settings.zoomFactor = fluidState.computedZoomFactor * 0.75f;
        fluidSettings.showSlice = false ;
        fluidSettings.sliceType = FluidMechanics.SLICE_STYLUS ;
        fluidSettings.constrainSelection = false ;
        updateSettings();
        mDatasetLoaded = true;
        requestRender();

    }

        @Override
        public void onCreateContextMenu(ContextMenu menu, View v, ContextMenuInfo menuInfo) {
            super.onCreateContextMenu(menu, v, menuInfo);
            menu.add(Menu.NONE, 0, Menu.NONE, "FTLElog");
            menu.add(Menu.NONE, 1, Menu.NONE, "Iron prot");
            menu.add(Menu.NONE, 2, Menu.NONE, "Head");
            menu.add(Menu.NONE, 3, Menu.NONE, "FTLE + velocity");
        }

        @Override
        public boolean onContextItemSelected(MenuItem item) {
            /*MainActivity.this.*/loadDataset(item.getItemId());
            return super.onContextItemSelected(item);
        }

    private void setupActionBar() {
        // Remove the title text and icon from the action bar
        getActionBar().setDisplayShowTitleEnabled(false);
        getActionBar().setDisplayShowHomeEnabled(false);
        getActionBar().setBackgroundDrawable(getResources().getDrawable(R.drawable.actionbar_bg));
    }

    private float dpToPixels(float dp) {
        Resources r = getResources();
        return TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, dp, r.getDisplayMetrics());
    }

    @SuppressWarnings("deprecation")
    private void setupSlider() {
        // "Jet" color map
        int[] colors = new int[] {
            0xFF00007F, // dark blue
            0xFF0000FF, // blue
            0xFF007FFF, // azure
            0xFF00FFFF, // cyan
            0xFF7FFF7F, // light green
            0xFFFFFF00, // yellow
            0xFFFF7F00, // orange
            0xFFFF0000, // red
            0xFF7F0000  // dark red
        };
        GradientDrawable colormap = new GradientDrawable(GradientDrawable.Orientation.BOTTOM_TOP, colors);
        colormap.setGradientType(GradientDrawable.LINEAR_GRADIENT);
        final VerticalSeekBar slider = (VerticalSeekBar)findViewById(R.id.verticalSlider);
        slider.setBackgroundDrawable(colormap);
        slider.setProgressDrawable(new ColorDrawable(0x00000000)); // transparent

        slider.setProgress((int)(fluidSettings.surfacePercentage * 100));

        final TextView sliderTooltip = (TextView)findViewById(R.id.sliderTooltip);
        sliderTooltip.setVisibility(View.INVISIBLE);

        slider.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            double mProgress = -1;
            boolean mPressed = false;

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
                sliderTooltip.setVisibility(View.VISIBLE);
                mPressed = true;
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
                sliderTooltip.setVisibility(View.INVISIBLE);
                if (mProgress != -1) {
                    // Log.d(TAG, "setSurfaceValue " + mProgress);
                    fluidSettings.surfacePreview = false;
                    fluidSettings.surfacePercentage = mProgress;
                    updateDataSettings();
                    mProgress = -1;
                }
                mPressed = false;
			}

			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                // Only handle events called from VerticalSeekBar. Other events, generated
                // from the base class SeekBar, contain bogus values because the Android
                // SeekBar was not meant to be vertical.
                if (fromUser)
                    return;

                if (!mPressed)
                    return;

                sliderTooltip.setText(progress + "%");
                mProgress = (double)progress/seekBar.getMax();
                // Log.d(TAG, "mProgress = " + mProgress);
                int pos = seekBar.getTop() + (int)(seekBar.getHeight() * (1.0 - mProgress));
                sliderTooltip.setPadding((int)dpToPixels(5), 0, (int)dpToPixels(5), 0);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(sliderTooltip.getLayoutParams());
                lp.topMargin = pos - sliderTooltip.getHeight()/2;
                lp.rightMargin = (int)dpToPixels(25);
                sliderTooltip.setLayoutParams(lp);

                
                fluidSettings.surfacePreview = true;
                fluidSettings.surfacePercentage = mProgress;
                updateDataSettings();
                
			}
		});
    }


    private void setupSliderPrecision() {
        // "Jet" color map
        int[] colors = new int[] {
            0xFF00007F, // dark blue
            0xFF0000FF, // blue
            0xFF007FFF, // azure
            0xFF00FFFF, // cyan
            0xFF7FFF7F, // light green
            0xFFFFFF00, // yellow
            0xFFFF7F00, // orange
            0xFFFF0000, // red
            0xFF7F0000  // dark red
        };
        //Have to use int
        final int step = 1;
        final int max = 150;
        final int min = 10;
        final int initialValue = 100 ;
        final double initialPosition = 100 ;

        GradientDrawable colormap = new GradientDrawable(GradientDrawable.Orientation.BOTTOM_TOP, colors);
        colormap.setGradientType(GradientDrawable.LINEAR_GRADIENT);
        final VerticalSeekBar sliderPrecision = (VerticalSeekBar)findViewById(R.id.verticalSliderPrecision);
        //sliderPrecision.setBackgroundDrawable(colormap);
        //sliderPrecision.setProgressDrawable(new ColorDrawable(0x00000000)); // transparent

        sliderPrecision.setMax( (max - min) / step );
        sliderPrecision.setProgress((int)(initialPosition));

        final TextView sliderTooltipPrecision = (TextView)findViewById(R.id.sliderTooltipPrecision);
        sliderTooltipPrecision.setVisibility(View.INVISIBLE);

        sliderPrecision.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            double mProgress = -1;
            boolean mPressed = false;

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                sliderTooltipPrecision.setVisibility(View.VISIBLE);
                mPressed = true;
                //Log.d(TAG, "Precision Java = " + mProgress);
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                sliderTooltipPrecision.setVisibility(View.INVISIBLE);
                if (mProgress != -1) {
                    // Log.d(TAG, "setSurfaceValue " + mProgress);
                    //fluidSettings.precision = (float)mProgress;
                    //updateDataSettings();
                    mProgress = -1;
                }
                //sliderPrecision.setMax(0);
                //sliderPrecision.setProgress(0);

                //If we want the precision to be system-controlled
                mPressed = false;

                //If we want the precision to be user-controlled
            }

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                // Only handle events called from VerticalSeekBar. Other events, generated
                // from the base class SeekBar, contain bogus values because the Android
                // SeekBar was not meant to be vertical.
                
                mProgress = (double)progress/seekBar.getMax();
                // Log.d(TAG, "mProgress = " + mProgress);
                int pos = seekBar.getTop() + (int)(seekBar.getHeight() * (1.0 - mProgress));
                sliderTooltipPrecision.setPadding((int)dpToPixels(5), 0, (int)dpToPixels(5), 0);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(sliderTooltipPrecision.getLayoutParams());
                lp.topMargin = pos - sliderTooltipPrecision.getHeight()/2;
                lp.rightMargin = (int)dpToPixels(25);
                sliderTooltipPrecision.setLayoutParams(lp);

                //fluidSettings.surfacePreview = true;
                double value = min/(100.0) + (progress * (step/100.0));
                fluidSettings.precision = (float)value ;

                DecimalFormat df = new DecimalFormat("0.00##");
                String tooltipvalue = df.format(value);
                sliderTooltipPrecision.setText(tooltipvalue + "");
                updateDataSettings();
                //Log.d(TAG, "Precision Java = " + mProgress);
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main_menu, menu);

        mAxisClippingMenuItem = menu.findItem(R.id.action_axisClipping);
        mStylusClippingMenuItem = menu.findItem(R.id.action_stylusClipping);

        if (!menuInitialized) {
            fluidSettings.showVolume = menu.findItem(R.id.action_showVolume).isChecked();
            fluidSettings.showSurface = menu.findItem(R.id.action_showSurface).isChecked();
            fluidSettings.showStylus = true;
            fluidSettings.showSlice = menu.findItem(R.id.action_showSlice).isChecked();
            fluidSettings.showOutline = menu.findItem(R.id.action_showOutline).isChecked();
            settings.showCamera = menu.findItem(R.id.action_showCamera).isChecked();
            fluidSettings.showCrossingLines = menu.findItem(R.id.action_showLines).isChecked();

            updateSettings();
            updateDataSettings();

            menuInitialized = true;

        } else {
            menu.findItem(R.id.action_showVolume).setChecked(fluidSettings.showVolume);
            menu.findItem(R.id.action_showSurface).setChecked(fluidSettings.showSurface);
            menu.findItem(R.id.action_showSlice).setChecked(fluidSettings.showSlice);
            menu.findItem(R.id.action_showCamera).setChecked(settings.showCamera);
            menu.findItem(R.id.action_showLines).setChecked(fluidSettings.showCrossingLines);
            menu.findItem(R.id.action_showOutline).setChecked(fluidSettings.showOutline);
            menu.findItem(R.id.action_axisClipping).setChecked(fluidSettings.sliceType == FluidMechanics.SLICE_AXIS);
            menu.findItem(R.id.action_stylusClipping).setChecked(fluidSettings.sliceType == FluidMechanics.SLICE_STYLUS);
        }

        return true;
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        menu.findItem(R.id.action_clipDist).setEnabled(
            !menu.findItem(R.id.action_stylusClipping).isChecked()
            && menu.findItem(R.id.action_showSlice).isChecked()
        );
        menu.findItem(R.id.action_axisClipping).setEnabled(
            menu.findItem(R.id.action_showSlice).isChecked()
        );
        menu.findItem(R.id.action_stylusClipping).setEnabled(
            menu.findItem(R.id.action_showSlice).isChecked()
        );
        return true;
    }

    @Override
	public boolean onOptionsItemSelected(MenuItem item) {
        boolean handledSetting = false;
        boolean handledDataSetting = false;
        int tmp ;

		switch (item.getItemId()) {

// Menu show
            case R.id.action_showVolume:
                fluidSettings.showVolume = !fluidSettings.showVolume;
                item.setChecked(fluidSettings.showVolume);
                handledDataSetting = true;
                break;

            case R.id.action_showSurface:
                fluidSettings.showSurface = !fluidSettings.showSurface;
                item.setChecked(fluidSettings.showSurface);
                handledDataSetting = true;
                break;

            case R.id.action_showSlice:
                fluidSettings.showSlice = !fluidSettings.showSlice;
                item.setChecked(fluidSettings.showSlice);
                handledDataSetting = true;
                break;

            case R.id.action_showOutline:
                fluidSettings.showOutline = !fluidSettings.showOutline;
                item.setChecked(fluidSettings.showOutline);
                handledDataSetting = true ;
                break;

            case R.id.action_showCamera:
                settings.showCamera = !settings.showCamera;
                item.setChecked(settings.showCamera);
                handledSetting = true;
                break;

            case R.id.action_showLines:
                fluidSettings.showCrossingLines = !fluidSettings.showCrossingLines;
                item.setChecked(fluidSettings.showCrossingLines);
                handledDataSetting = true;
                break;

//End Menu Show
//Start Menu Action
            case R.id.action_axisClipping:
                item.setChecked(!item.isChecked());
                if (item.isChecked() && mStylusClippingMenuItem.isChecked())
                    mStylusClippingMenuItem.setChecked(false);
                handledDataSetting = true;
                break;

            case R.id.action_stylusClipping:
                item.setChecked(!item.isChecked());
                if (item.isChecked() && mAxisClippingMenuItem.isChecked())
                    mAxisClippingMenuItem.setChecked(false);
                handledDataSetting = true;
                break;

            case R.id.action_clipDist:
                showDistanceDialog();
                break;
            case R.id.change_IP:
                changeIP();
                break ;

            case R.id.action_reset:
                Log.d(TAG,"Reset");
                reset();
                break;

            case R.id.action_bluetooth:
                item.setChecked(!item.isChecked());
                if (item.isChecked()){
                    connectBle();
                }
                break ;

            case R.id.action_quit:
                Log.d(TAG,"Quit");
                //writeLogging();
                closeFile();
                //this.finish();
                //android.os.Process.killProcess(android.os.Process.myPid());
                System.exit(0);
                break ;

            /* Dataset */
            case R.id.action_dataset_ftle:
                loadDataset(ftle);
                reset();
                break ;
            case R.id.action_dataset_head:
                loadDataset(head);
                reset();
                break ;
            case R.id.action_dataset_ironprot:
                loadDataset(ironprot);
                reset();
                break ;
            case R.id.action_dataset_velocities:
                loadDataset(velocities);
                reset();
                break ;




        }

        if (handledSetting) {
            updateSettings();
            return true;
        } else if (handledDataSetting) {
            updateDataSettings();
            return true;
        } else {
            return super.onOptionsItemSelected(item);
        }
    }

    private void reset(){
        FluidMechanics.reset();
        isTouchOn = true ;
        //fluidSettings.dataORplane = 0 ; //Data 
        
        this.interactionMode = nothing ;
        //this.tangibleToggle.setChecked(false);
        //this.touchToggle.setChecked(false);
        //this.dataORplaneTangibleToggle.setChecked(true);
        //this.dataORplaneTouchToggle.setChecked(true);


        isTangibleOn = true ;
        interactionMode = dataTouchTangible ;
        dataORplaneTangible = true ; //Data
        dataORplaneTouch = true ;    //Data
        fluidSettings.precision = 1 ;
        fluidSettings.constrainSelection = false ;

        this.nbOfResets += 1 ;
        updateDataSettings();
        requestRender();

    }

    private void updateConstraintX(){
        if(constrainX){
            constrainY = false ;
            constrainZ = false ;
            fluidSettings.considerX = 1 ;
            fluidSettings.considerY = 0 ;
            fluidSettings.considerZ = 0 ;
            isConstrained = true ;
            client.considerX = 1 ;
            client.considerY = 0 ;
            client.considerZ = 0 ;
        }
        else if(!constrainX){

            fluidSettings.considerX = 1 ;
            fluidSettings.considerY = 1 ;
            fluidSettings.considerZ = 1 ;
            client.considerX = 1 ;
            client.considerY = 1 ;
            client.considerZ = 1 ;
            isConstrained = false ;
        }
        Log.d(TAG,"X Constraint updated");
        updateDataSettings();
    }

    private void updateConstraintY(){
        if(constrainY){
            constrainX = false ;
            constrainZ = false ;
            fluidSettings.considerX = 0 ;
            fluidSettings.considerY = 1 ;
            fluidSettings.considerZ = 0 ;
            client.considerX = 0 ;
            client.considerY = 1 ;
            client.considerZ = 0 ;
            isConstrained = true ;
        }
        else if(!constrainY){
            fluidSettings.considerX = 1 ;
            fluidSettings.considerY = 1 ;
            fluidSettings.considerZ = 1 ;
            isConstrained = false ;
            client.considerX = 1 ;
            client.considerY = 1 ;
            client.considerZ = 1 ;
        }
        Log.d(TAG,"Y Constraint updated");
        updateDataSettings();
    }

    private void updateConstraintZ(){
        if(constrainZ){
            constrainX = false ;
            constrainY = false ;
            fluidSettings.considerX = 0 ;
            fluidSettings.considerY = 0 ;
            fluidSettings.considerZ = 1 ;
            client.considerX = 0 ;
            client.considerY = 0 ;
            client.considerZ = 1 ;
            isConstrained = true ;
        }
        else if(!constrainZ){
            fluidSettings.considerX = 1 ;
            fluidSettings.considerY = 1 ;
            fluidSettings.considerZ = 1 ;
            client.considerX = 1 ;
            client.considerY = 1 ;
            client.considerZ = 1 ;
            isConstrained = false ;
        }
        Log.d(TAG,"Z Constraint updated");
        updateDataSettings();
    }

    private void updateConstraintAuto(){
        int tmp = (this.autoConstraint) ? 1 : 0; 
        int rotationValue = (this.autoConstraint) ? 0 : 1; 
        //fluidSettings.considerX = tmp ;
        //fluidSettings.considerY = tmp ;
        //fluidSettings.considerZ = tmp ;
        fluidSettings.considerRotation = rotationValue ;
        //fluidSettings.considerTranslation = tmp ;
        Log.d(TAG,"Auto Constraint updated. Value = "+this.autoConstraint);
        isConstrained = autoConstraint ;
        fluidSettings.autoConstraint = autoConstraint ;
        updateDataSettings();
        //Log.d(TAG,"X = "+fluidSettings.considerX+"  -- Y = "+fluidSettings.considerY
                        //+"  -- Z = "+ fluidSettings.considerZ+"  - Rotation  = "+fluidSettings.considerRotation);
    }

    private void showDistanceDialog() {
        LayoutInflater inflater = getLayoutInflater();
        View layout = inflater.inflate(R.layout.seekbar_dialog, null);

        SeekBar seekbar = (SeekBar)layout.findViewById(R.id.seekbar_dialog_seekbar);
        final TextView text = (TextView)layout.findViewById(R.id.seekbar_dialog_text);

        seekbar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // Log.d(TAG, "tracking started");
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // Log.d(TAG, "tracking stopped");
            }

            @Override
            public void onProgressChanged(SeekBar seekbar, int progress, boolean fromUser) {
                // Log.d(TAG, "progress changed: " + progress + " (fromUser = " + fromUser + ")");
                text.setText("Distance: " + progress);
                if (fromUser) {
                    fluidSettings.clipDist = progress;
                    updateDataSettings();
                }
            }
        });

        seekbar.setMax(2000);
        seekbar.setProgress((int)fluidSettings.clipDist);

        new AlertDialog.Builder(this)
            .setView(layout)
            .setTitle("Slicing distance")
            .setPositiveButton("Close", null)
            .show();
    }

    private void updateDataState() {
        FluidMechanics.getState(fluidState);
    }

    private void updateSettings() {
        NativeApp.setSettings(settings);
    }

    private void updateDataSettings() {
        fluidSettings.sliceType =
            mStylusClippingMenuItem.isChecked() ? FluidMechanics.SLICE_STYLUS
            : mAxisClippingMenuItem.isChecked() ? FluidMechanics.SLICE_AXIS
            : FluidMechanics.SLICE_CAMERA;

        FluidMechanics.setSettings(fluidSettings);

        VerticalSeekBar verticalSlider = (VerticalSeekBar)findViewById(R.id.verticalSlider);
        if (verticalSlider != null) {
            verticalSlider.setVisibility(
                fluidSettings.showSurface ? View.VISIBLE : View.INVISIBLE);
        }
    }

    @Override
    protected void onFrameProcessed() {
        FluidMechanics.getState(fluidState);
    }

    @Override
    public void onBackPressed() {
        // (ignore the back button)
    }

    private boolean isButton(View v){
        int id = v.getId();
        if(id == R.id.tangibleBtn /*|| id == R.id.constrainX ||
           id == R.id.constrainY || id == R.id.constrainZ || id == R.id.autoConstrain*/){

                return true ;
        }
        return false ;
    }

    private boolean isOnTouchButton(float x, float y){
        if(x < 270){
            //Log.d("Seed or Tangible");
            return true ;
        }
        else if(x<=500 && y>=820){
            //Log.d(TAG,"Buttons on the left");
            return true ;
        }
        else if(x>=1500 && y >=820){
            //Log.d(TAG,"Buttons on the right");
            return true ;
        }
        return false ;
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        //First compute RAW coordinates of each touch event
        float rawPosX[] = new float[5];
        float rawPosY[] = new float[5];
        float ofsX = event.getRawX() - event.getX();
        float ofsY = event.getRawY() - event.getY();
        boolean removedButtonFinger = false ;
        for (int i = 0; i < event.getPointerCount(); i++) {
            rawPosX[i] = event.getX(i) + ofsX;
            rawPosY[i] = event.getY(i) + ofsY;
            //isOnTouchButton(rawPosX[i],rawPosY[i]);
            //Log.d(TAG,"Finger "+i+" X = "+rawPosX[i]+" Y = "+rawPosY[i]);
        }


        if(v.getId() == R.id.tangibleBtn){
            int index = event.getActionIndex();
            //Log.d(TAG,"INDEX = "+fingerOnButtonIndex);
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                isTangiblePressed = true ;
                FluidMechanics.buttonPressed();
                this.tangibleBtn.setPressed(true);
                this.nbOfFingersButton+=1 ;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                FluidMechanics.buttonReleased();
                isTangiblePressed = false ;
                this.tangibleBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            
            //return true ;
        }

        /*else if(v.getId() == R.id.constrainX){
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                constrainX = true ;
                this.constrainXBtn.setPressed(true);
                this.nbOfFingersButton+=1;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                constrainX = false ;
                this.constrainXBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            //Log.d(TAG,"Touched constrainX");
            updateConstraintX();
            int index = event.getActionIndex();
            //return true ;
        }

        else if(v.getId() == R.id.constrainY){
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                constrainY = true ;
                this.constrainYBtn.setPressed(true);
                this.nbOfFingersButton+=1;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                constrainY = false ;
                this.constrainYBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            //Log.d(TAG,"Touched constrainY");
            updateConstraintY();
            int index = event.getActionIndex();
            //return true ;
        }

        else if(v.getId() == R.id.constrainZ){
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                constrainZ = true ;
                this.constrainZBtn.setPressed(true);
                this.nbOfFingersButton+=1;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                constrainZ = false ;
                this.constrainZBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            //Log.d(TAG,"Touched constrainZ");
            updateConstraintZ();
            int index = event.getActionIndex();
            //return true ;
        }

        else if(v.getId() == R.id.autoConstrain ){
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                this.autoConstraint = true ;
                this.autoConstrainBtn.setPressed(true);
                this.nbOfFingersButton+=1;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                this.autoConstraint = false ;
                this.autoConstrainBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            //Log.d(TAG,"Touched constrain Auto");
            updateConstraintAuto();
            int index = event.getActionIndex();
            //return true ;
        }*/

        else if(v.getId() == R.id.seedingBtn){
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                fluidSettings.isSeeding = true ;
                this.seedingBtn.setPressed(true);
                this.nbOfFingersButton+=1;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                fluidSettings.isSeeding = false ;
                this.seedingBtn.setPressed(false);
                FluidMechanics.resetParticles();
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
                client.seedPoint = "-1000000.0;-1000000.0;-1000000.0";
                client.valuesupdated = true ;
            }
            updateDataSettings();
            //return true ;
        }



        //Log.d(TAG,"X = "+fluidSettings.considerX+"  -- Y = "+fluidSettings.considerY
        //                +"  -- Z = "+ fluidSettings.considerZ+"  - Rotation  = "+fluidSettings.considerRotation);
        mGestureDetector.onTouchEvent(event);

        //Log.d(TAG, "View == "+(v.getId() == R.id.glSurfaceView));
        if(mDatasetLoaded ){
            this.interactionType = touchInteraction ;
            switch (event.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_POINTER_DOWN:
                {   
                    int index = event.getActionIndex();
                    int id = event.getPointerId(index);
                    //Log.d("Finger ID", "Finger ID = "+id);
                    //Log.d("Finger Index", "Finger Index = "+index);
                    //Log.d(TAG, "Finger X = "+event.getRawX()+" Finger Y = "+event.getRawY());
                    if(isOnTouchButton(rawPosX[index], rawPosY[index]) == false){
                        //Log.d(TAG, "Add Finger X = "+event.getX(index)+"  Y = "+event.getY(index));
                        Log.d(TAG, "Add Finger X = "+rawPosX[index]+"  Y = "+rawPosY[index]);
                        this.nbOfFingers +=1 ;
                        //FluidMechanics.addFinger(event.getX(index), event.getY(index), id);    
                        FluidMechanics.addFinger(rawPosX[index], rawPosY[index], id);    
                    }
                    break ;
                }

                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_POINTER_UP:
                {
                    int index = event.getActionIndex();
                    int id = event.getPointerId(index);
                    //Log.d("Finger ID", "Finger ID = "+id);
                    //Log.d("Finger Index", "Finger Index = "+index);
                    //if(isOnTouchButton(rawPosX[index], rawPosY[index]) == false){
                        //Log.d(TAG, "Remove Finger");
                        FluidMechanics.removeFinger(id);
                        if(removedButtonFinger == false ){
                            this.nbOfFingers -= 1 ;    
                        }
                        
                    //}
                    break ;
                }

                case MotionEvent.ACTION_MOVE:
                {
                    int numPtrs = event.getPointerCount();
                    float [] xPos = new float[numPtrs];
                    float [] yPos = new float[numPtrs];
                    int [] ids = new int[numPtrs];
                    for (int i = 0; i < numPtrs; ++i)
                    {
                        ids[i]  = event.getPointerId(i);
                        xPos[i] = event.getX(i);
                        yPos[i] = event.getY(i);
                            //FluidMechanics.updateFingerPositions(xPos[i],yPos[i],ids[i]);
                            FluidMechanics.updateFingerPositions(rawPosX[i], rawPosY[i], ids[i]);  
                        
                    }
                    break ;
                }
            }

            this.isInteracting = true ;
            requestRender();
            Log.d(TAG,"NB OF FINGERS = "+this.nbOfFingers);
            Log.d(TAG,"Nb of fingers button = "+this.nbOfFingersButton);
            // NativeApp.setZoom(mZoomFactor);

        }

        return true;
    }

    @Override
    public boolean onDoubleTap(MotionEvent e) {
        return true;
    }

    @Override
    public boolean onDoubleTapEvent(MotionEvent e) {
        // Log.d(TAG, "onDoubleTapEvent");
        return true;
    }

    @Override
    public boolean onSingleTapConfirmed(MotionEvent e) {
        return true;
    }

   public void requestRender(){
        if (mView != null){
            //Log.d(TAG,"RequestRender");
			String s = FluidMechanics.getSelectionData();
			client.setSelectionData(s);
			client.setData(FluidMechanics.getData());
			client.setPostTreatment(FluidMechanics.getPostTreatmentMatrix());
			client.setSubData(FluidMechanics.getSubData());
            mView.requestRender();
        }
   } 

   public void changeIP(){
        this.client.closeConnection = false ;
        this.client = new Client();
        this.client.execute();

   }

   public void changeInteractionMode(int mode){
        this.interactionMode = mode ;
        FluidMechanics.setInteractionMode(mode);
   }



    @Override
    public void onClick(View v) {
        if(v.getId() == R.id.resetBtn){
            fluidSettings.reset = true ;
            updateSettings();
			client.setSelectionData("3");
			FluidMechanics.reset();
        }
    }

    //Bluetooth

    final private static int STATE_BLUETOOTH_OFF    = 1;
    final private static int STATE_DISCONNECTED     = 2;
    final private static int STATE_CONNECTING       = 3;
    final private static int STATE_CONNECTED        = 4;

    private int state;

    private boolean scanStarted;
    private boolean scanning;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothDevice bluetoothDevice;
    private EditData valueEdit;
    private RFduinoService rfduinoService;
    private String data ;


    private final BroadcastReceiver bluetoothStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);
            if (state == BluetoothAdapter.STATE_ON) {
                upgradeState(STATE_DISCONNECTED);
            } else if (state == BluetoothAdapter.STATE_OFF) {
                downgradeState(STATE_BLUETOOTH_OFF);
            }
        }
    };

    private final BroadcastReceiver scanModeReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            scanning = (bluetoothAdapter.getScanMode() != BluetoothAdapter.SCAN_MODE_NONE);
            scanStarted &= scanning;
            //updateUi();
        }
    };


    private final ServiceConnection rfduinoServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            rfduinoService = ((RFduinoService.LocalBinder) service).getService();
            if (rfduinoService.initialize()) {
                if (rfduinoService.connect(bluetoothDevice.getAddress())) {
                    upgradeState(STATE_CONNECTING);
                }
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            rfduinoService = null;
            downgradeState(STATE_DISCONNECTED);
        }
    };

    private final BroadcastReceiver rfduinoReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (RFduinoService.ACTION_CONNECTED.equals(action)) {
                upgradeState(STATE_CONNECTED);
            } else if (RFduinoService.ACTION_DISCONNECTED.equals(action)) {
                downgradeState(STATE_DISCONNECTED);
            } else if (RFduinoService.ACTION_DATA_AVAILABLE.equals(action)) {
                addData(intent.getByteArrayExtra(RFduinoService.EXTRA_DATA));
            }
        }
    };

    private void addData(byte[] data) {
        //TODO
    }

    @Override
    protected void onStart() {
        super.onStart();

        registerReceiver(scanModeReceiver, new IntentFilter(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED));
        registerReceiver(bluetoothStateReceiver, new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED));
        registerReceiver(rfduinoReceiver, RFduinoService.getIntentFilter());

        updateState(bluetoothAdapter.isEnabled() ? STATE_DISCONNECTED : STATE_BLUETOOTH_OFF);
    }


    private void upgradeState(int newState) {
        if (newState > state) {
            updateState(newState);
        }
    }

    private void downgradeState(int newState) {
        if (newState < state) {
            updateState(newState);
        }
    }

    private void updateState(int newState) {
        state = newState;
        Log.d(TAG,"STATE updated");
        /*switch (newState){
            case STATE_BLUETOOTH_OFF:
                bluetoothState.setText("STATE_BLUETOOTH_OFF");
                break ;
            case STATE_DISCONNECTED:
                bluetoothState.setText("STATE_DISCONNECTED");
                break ;
            case STATE_CONNECTING:
                bluetoothState.setText("STATE_CONNECTING");
                break ;
            case STATE_CONNECTED:
                bluetoothState.setText("STATE_CONNECTED");
                break ;
        }*/
        updateConnectionProcess();
        
    }


    protected void scan(){
        scanStarted = true;
        Log.d(TAG, "SCANNNNNNNNNNNNNN");
        bluetoothAdapter.startLeScan(new UUID[]{ RFduinoService.UUID_SERVICE },MainActivity.this);
        updateConnectionProcess();
    }

    protected void connect(){
        Intent rfduinoIntent = new Intent(MainActivity.this, RFduinoService.class);
        bindService(rfduinoIntent, rfduinoServiceConnection, BIND_AUTO_CREATE);
        updateConnectionProcess();
        Log.d(TAG, "CONNECT");
    }

    @Override
    public void onLeScan(BluetoothDevice device, final int rssi, final byte[] scanRecord) {
    Log.d(TAG, "onLeScan");
        bluetoothAdapter.stopLeScan(this);
        bluetoothDevice = device;

        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                updateConnectionProcess();
            }
        });
    }


    private void updateConnectionProcess() {

        Log.d(TAG, "updateConnectionProcess state=" + state);
    
        if (state == STATE_DISCONNECTED) {
            Log.d(TAG, "state_disconnected bluetoothDevice=" + bluetoothDevice + " scanStarted=" + scanStarted);
            if (bluetoothDevice == null) {
                if (!scanStarted) {
                    scanStarted = true;
                    Log.d(TAG, "starting scan");
                    bluetoothAdapter.startLeScan(new UUID[] { RFduinoService.UUID_SERVICE }, this);
                }
            } else {
                Intent rfduinoIntent = new Intent(this, RFduinoService.class);
                Log.d(TAG, "bindService");
                bindService(rfduinoIntent, rfduinoServiceConnection, BIND_AUTO_CREATE);
            }
        }
    }

    protected void createBluetooth(){
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		 Log.d(TAG, "toggling bluetooth...");
    }

    protected void connectBle(){
        scan();
        connect();
    }


}
