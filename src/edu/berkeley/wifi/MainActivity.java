package edu.berkeley.wifi;

import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;

import org.json.JSONException;
import org.json.JSONObject;

import pf.floors.Area;
import pf.floors.AreaBuilder;
import pf.particle.ParticlePosition;
import pf.utils.Point2D;
import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.hardware.Camera;
import android.hardware.Camera.PictureCallback;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.provider.MediaStore;
import android.util.Log;
import android.view.Menu;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import cz.muni.fi.sandbox.dsp.filters.ContinuousConvolution;
import cz.muni.fi.sandbox.dsp.filters.FrequencyCounter;
import cz.muni.fi.sandbox.dsp.filters.SinXPiWindow;
import cz.muni.fi.sandbox.service.stepdetector.MovingAverageStepDetector;
import cz.muni.fi.sandbox.service.stepdetector.MovingAverageStepDetector.MovingAverageStepDetectorState;

public class MainActivity extends Activity implements SensorEventListener{
	static final String IMAGE_URL = "http://sofia.eecs.berkeley.edu:8001";
	private static int TAKE_PICTURE = 1;    
	private SensorManager mSensorManager;
	private Sensor mCompass;
	private Sensor linearAccelerometer, rotationSensor, accelerometer, magnetometer;
	private Camera camera;
	private CameraPreview mPreview;
	private TextView mTextView;
	private MapView mMapView;
	private ImageView mImgLocView;
	private ParticlePosition mParticleCloud;
	private AreaBuilder mCory2Builder;
	private Area mCory2;
	private String wifiCoordsFile;
	String imgFile;
	float[] inR = new float[16];
    float[] I = new float[16];
    float[] gravity = new float[3];
    float[] geomag = new float[3];
    float[] orientVals = new float[3];
    private float[] cameraPose = new float[3];
    float last_accel = 0f;
    
    final float pi = (float) Math.PI;
    final float rad2deg = 180/pi;    

    private MovingAverageStepDetector mStepDetector;
    private ContinuousConvolution mCC;
    private FrequencyCounter freqCounter;

    double movingAverage1 = MovingAverageStepDetector.MA1_WINDOW;
    double movingAverage2 = MovingAverageStepDetector.MA2_WINDOW;

    double lowPowerCutoff = MovingAverageStepDetector.LOW_POWER_CUTOFF_VALUE;
    double highPowerCutoff = MovingAverageStepDetector.HIGH_POWER_CUTOFF_VALUE;

    private int mMASize = 20;
    @SuppressWarnings("unused")
    private float mSpeed = 1f;
    float mImgHeading;
    long tImgHeadingTaken = 0L;
    float mConvolution, mLastConvolution;

    double stepLength = -10.0;
    ArrayList<Point2D> imgStepHistory;
    double[] cloudCenter;
    Intent intent;
    private WifiScanService wifiService;
	private PictureCallback mPicture = new PictureCallback() {
		
	    @Override
	    public void onPictureTaken(byte[] data, Camera camera) {
	    	Bitmap tmp;
	    	try {
				tmp = createBitmap(data, 1280, 1024, 90);
			} catch (FileNotFoundException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
				return;
			}
	    	ByteArrayOutputStream stream = new ByteArrayOutputStream();
	    	tmp.compress(Bitmap.CompressFormat.JPEG, 100, stream);
	    	data = stream.toByteArray();
	    	File root = new File(Environment.getExternalStorageDirectory()+File.separator+"wifiloc");
		    File output = new File(root, "img_" + System.currentTimeMillis() + ".jpg");
		    FileOutputStream fos;
			try {
				fos = new FileOutputStream(output);
				fos.write(data);
				fos.close();
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
	        
	        JSONObject pose, returnParams;
			JSONObject imageQuery;
	        HashMap<String, Float> poseMap = new HashMap<String, Float>();
			HashMap<String, Boolean> returnMap = new HashMap<String, Boolean>();
			HashMap<String, Object> paramsMap = new HashMap<String, Object>();
			
			HashMap<String, Object> imageQueryMap = new HashMap<String, Object>();
	        poseMap.put("latitude", 0f);
			poseMap.put("longitude", 0f);
			poseMap.put("altitude", (float)0.0);
			poseMap.put("yaw", cameraPose[0]);
			poseMap.put("pitch", cameraPose[1]);
			poseMap.put("roll", cameraPose[2]);
			poseMap.put("ambiguity_meters", (float)1.0e+12);
			pose = new JSONObject(poseMap);

			returnMap.put("statistics", true);
			returnMap.put("image_data", false);
			returnMap.put("estimated_client_pose", true);
			returnMap.put("pose_visualization_only", false);
			returnParams = new JSONObject(returnMap);
			
			paramsMap.put("method", "client_query");
			paramsMap.put("user", "test");
			paramsMap.put("database", "0815_db");
			paramsMap.put("deadline_seconds", 60.0);
			paramsMap.put("disable_gpu", false);
			paramsMap.put("perfmode", "fast");
			paramsMap.put("pose", pose);
			paramsMap.put("return", returnParams);
			
			imageQueryMap.put("params", paramsMap);
			imageQuery = new JSONObject(paramsMap);
			QueryTask qr;
			Context ctx = getBaseContext();
			qr = new QueryTask(IMAGE_URL, imageQuery, "image", data);
			qr.execute(ctx); 
			//Toast.makeText(ctx, "Sent request to server", Toast.LENGTH_SHORT).show();
			writeToFile("timelog.txt", "sent " + System.currentTimeMillis() + "\n");
			camera.startPreview();
			
	    }
	    
	    private Bitmap createBitmap(byte[] imageData, int maxWidth, int maxHeight,
		        int rotationDegrees) throws FileNotFoundException {

		    BitmapFactory.Options options = new BitmapFactory.Options();
		    options.inSampleSize = 2;
		    options.inDensity = 240;
		    int imageWidth = 0;
		    int imageHeight = 0;

		    Bitmap image = BitmapFactory.decodeByteArray(imageData, 0,
		            imageData.length, options);

		    imageWidth = image.getWidth();
		    imageHeight = image.getHeight();

		    //if (imageWidth > maxWidth || imageHeight > maxHeight) {

		        double imageAspect = (double) imageWidth / imageHeight;
		        double desiredAspect = (double) maxWidth / maxHeight;
		        double scaleFactor;

		        if (imageAspect < desiredAspect) {
		            scaleFactor = (double) maxHeight / imageHeight;
		        } else {
		            scaleFactor = (double) maxWidth / imageWidth;
		        }

		        float scaleWidth = ((float) scaleFactor) * imageWidth;
		        float scaleHeight = ((float) scaleFactor) * imageHeight;

		        Bitmap scaledBitmap = Bitmap.createScaledBitmap(image,
		                (int) scaleWidth, (int) scaleHeight, true);
		        image = scaledBitmap;
		    //}

		    if (rotationDegrees != 0) {
		        Matrix matrix = new Matrix();
		        matrix.postRotate(90);
		        Bitmap rotatedBMP = Bitmap.createBitmap(scaledBitmap , 0, 0, scaledBitmap .getWidth(), scaledBitmap .getHeight(), matrix, true);

		       image = rotatedBMP;
		    }

		    return image;
		}
	};
    
	
	
	public void processAccelerometerEvent(SensorEvent event) {                
        mConvolution = (float) (mCC.process(event.values[2]));
        mStepDetector.onSensorChanged(event);
        displayStepDetectorState(mStepDetector);
}


	void displayStepDetectorState(MovingAverageStepDetector detector) {
		MovingAverageStepDetectorState s = detector.getState();
		boolean stepDetected = s.states[0];
		boolean signalPowerOutOfRange = s.states[1];

		/*
		TextView offset_degrees = (TextView) findViewById(R.id.offset_degrees);
		float offsetDeg = 0.0f;
		try {
			offsetDeg = Float.parseFloat(offset_degrees.getText().toString());
		}
		catch (Exception e) {
			e.printStackTrace();
		}*/
		// The offset is now hard coded
		float offsetDeg = 0.0f;
		
		
		
		if (stepDetected) {
			
		        if (!signalPowerOutOfRange && detector.stepLength >= 0) {
		        	/*Toast.makeText(getBaseContext(), 
		  		          "VALID STEP! " + detector.stepLength, 
		  		          Toast.LENGTH_SHORT).show();*/
		        		
		        		if (mParticleCloud != null) {
		        			float azimuth = orientVals[0]*rad2deg+12.387f-offsetDeg;
		        			float roll = orientVals[2]*rad2deg;
		    		        
		        			double currHeading = (double)azimuth;
		        			if (Math.abs(roll) > 90)
		        				currHeading = (currHeading+540)%360;
		        			mTextView.setText("Heading: " + currHeading);
		        			long currTime = System.currentTimeMillis();
		        			if (wifiService != null) {
		        				wifiService.addStep(new Step(currHeading, detector.stepLength, currTime));
		        				}
		        			this.mParticleCloud.onStep(currHeading, detector.stepLength);
		        			writeToFile("heading.txt", currTime + " " + currHeading + " "+ detector.stepLength + "\n");
				        	String partCenter = mParticleCloud.getCenter();
				        	//mTextView.setText(partCenter);
				        	String coords[] = partCenter.split("\\s+");
				        	float new_x = Float.parseFloat(coords[0]);
				        	float new_y = Float.parseFloat(coords[1]);
				        	imgStepHistory.add(new Point2D(new_x,new_y));
				        	mMapView.updatePos(new_x, new_y);
				        	long tstamp = System.currentTimeMillis();
				        	writeToFile("path.txt", "s" + " " + new_x + " " + new_y + " " + tstamp + " " + 0 + " " + 0 + " " + "\n");
				        	cloudCenter[0] = (double)new_x;
				        	cloudCenter[1] = (double)new_y;
				        	if (wifiService != null)
				        		wifiService.setCloudPosition(new Point2D(cloudCenter[0], cloudCenter[1]));
		           			}
		        		
		        	}
			}
		}
	// The following method is required by the SensorEventListener interface;
	public void onAccuracyChanged(Sensor sensor, int accuracy) {    
	}

	// The following method is required by the SensorEventListener interface;
	// Hook this event to process updates;
	public void onSensorChanged(SensorEvent event) {
		final float alpha = 0.8f;
		float[] linear_acceleration = {0f,0f,0f};
		int type = event.sensor.getType();
		// If the sensor data is unreliable return
		//if (event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE)
		 //   return;

	   // Gets the value of the sensor that has been changed
	   switch (type) {  
		   	case Sensor.TYPE_ACCELEROMETER:
		   		synchronized(this) {
		   			gravity = event.values.clone();
		   			//writeToFile("accel.txt","Data: " + linear_acceleration[1] + " ");
		   			cameraPose = gravity;
		   			processAccelerometerEvent(event);
                    freqCounter.push(event.timestamp);
                    float rate = freqCounter.getRateF();
                    if (rate != 0.0f)
                            mSpeed = 100f / rate;
		   		}
		   		break;
		   	case Sensor.TYPE_MAGNETIC_FIELD:
		   		geomag = event.values.clone();
		   		break;
		   }

		   // If gravity and geomag have values then find rotation matrix
		   if (gravity != null && geomag != null){

		    // checks that the rotation matrix is found
		    boolean success = SensorManager.getRotationMatrix(inR, I, gravity, geomag);
		    if (success){
		    	
		    	SensorManager.getOrientation(inR, orientVals);
		        float azimuth = orientVals[0]*rad2deg;
		        float pitch = orientVals[1]*rad2deg;
		        float roll = orientVals[2]*rad2deg;
	
		       }
		    }
	
	    
	}
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		registerReceiver(uiUpdated, new IntentFilter("LOCATION_UPDATED"));
		registerReceiver(uiUpdated_img, new IntentFilter("IMG_LOCATION_UPDATED"));
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
	    mTextView = (TextView) findViewById(R.id.tvSensor);
	    mMapView = (MapView) findViewById(R.id.map_view);
	    mImgLocView = (ImageView) findViewById(R.id.imgLocView);
	    linearAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        rotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        
        mStepDetector = new MovingAverageStepDetector(movingAverage1, movingAverage2, lowPowerCutoff, highPowerCutoff);

        mCC = new ContinuousConvolution(new SinXPiWindow(mMASize));
        freqCounter = new FrequencyCounter(20);
        cloudCenter = new double[2];        
        File directory = new File(Environment.getExternalStorageDirectory()+File.separator+"wifiloc");
        directory.mkdirs();
	}
	private ServiceConnection mConnection = new ServiceConnection() {

	    public void onServiceConnected(ComponentName className, 
	        IBinder binder) {
	      WifiScanService.MyBinder b = (WifiScanService.MyBinder) binder;
	      wifiService = b.getService();
	      Toast.makeText(MainActivity.this, "Connected", Toast.LENGTH_SHORT)
	          .show();
	    }

	    public void onServiceDisconnected(ComponentName className) {
	      wifiService = null;
	    }
	  };
	  
	@Override
	public void onDestroy(){
		super.onDestroy();
		Intent intent = new Intent(this, WifiScanService.class);
		stopService(intent);
		if (wifiService != null)
			unbindService(mConnection);
		unregisterReceiver(uiUpdated);
		unregisterReceiver(uiUpdated_img);
	}
	
	@Override
	public void onPause(){
		super.onPause();
		Intent intent = new Intent(this, WifiScanService.class);
		stopService(intent);
		PendingIntent pintent = PendingIntent.getService(this, 0, intent, 0);
		Calendar cal = Calendar.getInstance();
		AlarmManager alarm = (AlarmManager)getSystemService(Context.ALARM_SERVICE);
		alarm.cancel(pintent);
		if (camera != null) {
			camera.setPreviewCallback(null);
	        mPreview.getHolder().removeCallback(mPreview);
			camera.stopPreview();
			camera.release();
			}
		mSensorManager.unregisterListener(this, linearAccelerometer);
		mSensorManager.unregisterListener(this, rotationSensor);
		mSensorManager.unregisterListener(this, accelerometer);
		mSensorManager.unregisterListener(this, magnetometer);
	}

	@Override
	public void onResume() {
		super.onResume();
		camera = getCameraInstance();
		writeToFile("wifiscan.txt","*** NEW LOCATION ***\n");
		
		Camera.Parameters params = camera.getParameters();
		/*List<Camera.Size> psizes = params.getSupportedPictureSizes();
		Iterator<Camera.Size> iter = psizes.iterator();
		while (iter.hasNext()) {
			Camera.Size csz = iter.next();
			writeToFile("wifiscan.txt", csz.width + " " + csz.height +"\n");
			}*/
		/*List<String> fmodes = params.getSupportedSceneModes();
		Iterator<String> iterator = fmodes.iterator();
		while (iterator.hasNext())
			{
			String item = iterator.next();
			writeToFile("wifiscan.txt",item+"\n");
			}*/
		//String focusmode = params.getSceneMode();
		//writeToFile("wifiscan.txt",focusmode+"\n");
		//params.setFocusMode("steadyphoto");
		
		//params.set("orientation", "portrait");
		//params.set("rotation", 90);
		params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
		params.setSceneMode(Camera.Parameters.SCENE_MODE_STEADYPHOTO);
		params.setPictureSize(1280, 720);
		params.setExposureCompensation(-2);
		//camera.setParameters(params);
		
		this.setDisplayOrientation(camera, 90);
		mPreview = new CameraPreview(this, camera);
        FrameLayout preview = (FrameLayout) findViewById(R.id.camera_preview);
        preview.addView(mPreview);
        preview.setKeepScreenOn(true);
        camera.startPreview();
		mSensorManager.registerListener(this, linearAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_FASTEST);
       
        //FrameLayout mapview = (FrameLayout) findViewById(R.id.map_view); 
        this.mCory2Builder = new AreaBuilder();        
        AssetManager am = getAssets();
		this.mCory2Builder.readSimpleTextWalls(am, "cory2.edge");
		mCory2 = mCory2Builder.create();
		this.mParticleCloud = new ParticlePosition(0,0, mCory2); 
		mParticleCloud.readCoords(am, "wifi_coords.dat");
		//this.mParticleCloud.setArea(mCory2);
		imgStepHistory = new ArrayList<Point2D>();



		// Instead of letting user initiate scan, automate it.
		startScan();
        
	}
	protected void setDisplayOrientation(Camera camera, int angle){
	    Method downPolymorphic;
	    try
	    {
	        downPolymorphic = camera.getClass().getMethod("setDisplayOrientation", new Class[] { int.class });
	        if (downPolymorphic != null)
	            downPolymorphic.invoke(camera, new Object[] { angle });
	    }
	    catch (Exception e1)
	    {
	    }
	}
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	


	// Eliminated "view" argument
	public void startScan() {
		intent = new Intent(this, WifiScanService.class);




		// Right now only 2nd floor is enabled.
		/*
		RadioButton cory2nd = (RadioButton) findViewById(R.id.radio0);
	    RadioButton cory3rd = (RadioButton) findViewById(R.id.radio1);
	    
	    Bundle b = new Bundle();
	    if (cory2nd.isChecked()) {
	    	Log.d("FLOOR","1");
	    	b.putInt("floor_id", 1);
	    }
	    else {
	    	Log.d("FLOOR","2");
	    	b.putInt("floor_id", 2);
	    }*/
	    Bundle b = new Bundle();
	    b.putInt("floor_id", 1);




	    //b.putString("cutoff_freq", cutoff_freq.getText().toString());
	    //b.putDoubleArray("cloudCenter", cloudCenter);
	    intent.replaceExtras(b);
	    bindService(intent, mConnection, Context.BIND_AUTO_CREATE);
		PendingIntent pintent = PendingIntent.getService(this, 0, intent, PendingIntent.FLAG_CANCEL_CURRENT);
		Calendar cal = Calendar.getInstance();
		AlarmManager alarm = (AlarmManager)getSystemService(Context.ALARM_SERVICE);
		// Start every 5 seconds
		alarm.setRepeating(AlarmManager.RTC_WAKEUP, cal.getTimeInMillis(), 5*1000, pintent); 		
	}

	
	public Bitmap resizeBitmap(Bitmap img, double maxWidth, double maxHeight) {
		float imageWidth = img.getWidth();
	    float imageHeight = img.getHeight();

	    if (imageWidth > maxWidth || imageHeight > maxHeight) {

	        double imageAspect = (double) imageWidth / imageHeight;
	        double desiredAspect = (double) maxWidth / maxHeight;
	        double scaleFactor;

	        if (imageAspect < desiredAspect) {
	            scaleFactor = (double) maxHeight / imageHeight;
	        } else {
	            scaleFactor = (double) maxWidth / imageWidth;
	        }

	        float scaleWidth = ((float) scaleFactor) * imageWidth;
	        float scaleHeight = ((float) scaleFactor) * imageHeight;

	        Bitmap scaledBitmap = Bitmap.createScaledBitmap(img,
	                (int) scaleWidth, (int) scaleHeight, true);
	        
	        return scaledBitmap;
	    }
	    return img;
	}
	public void writeToFile(String fname, String data)  {
		File root = new File(Environment.getExternalStorageDirectory()+File.separator+"wifiloc");
	   
	    File file = new File(root, fname);
	    FileWriter filewriter;
		try {
			filewriter = new FileWriter(file,true);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			Log.d("IMGRES","Could not create file " + e1.getMessage());
			return;
		}
	     
	    BufferedWriter out = new BufferedWriter(filewriter);
			try {
			out.write(data);
			out.close();
			} catch (IOException e)
			{
			Log.d("IMGRES","Could not write to file " + e.getMessage());
			}
	}
	
	private void dispatchTakePictureIntent(int actionCode) {
		File root = new File(Environment.getExternalStorageDirectory()+File.separator+"wifiloc");
	    File output = new File(root, "img_" + System.currentTimeMillis() + ".jpg");
	    imgFile = new String(output.getAbsolutePath());
	    Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
	    takePictureIntent.putExtra(MediaStore.EXTRA_OUTPUT, Uri.fromFile(output));
	    startActivityForResult(takePictureIntent, actionCode);
	}
	
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
	    // Check which request we're responding to
	    if (requestCode == TAKE_PICTURE) {
	        // Make sure the request was successful
	        if (resultCode == RESULT_OK) {
	        	Context ctx = getBaseContext();
	        	byte[] imgData = null;
	        	
	        	try {
	        	RandomAccessFile f = new RandomAccessFile(imgFile, "r");
	        	imgData = new byte[(int)f.length()];
	        	f.read(imgData);
	        	f.close();
	        	} catch (Exception e) {Log.d("IMGLOC","Couldn't read " + imgFile);}
	    		/*try {
	    			imgData = IOUtil.readFile(imgFile);
	    		} catch (IOException e) {
	    			Log.d("IMGLOC","Couldn't read " + imgFile);
	    		}*/
	        	JSONObject pose, returnParams;
	    		JSONObject imageQuery;
	            HashMap<String, Float> poseMap = new HashMap<String, Float>();
	    		HashMap<String, Boolean> returnMap = new HashMap<String, Boolean>();
	    		HashMap<String, Object> paramsMap = new HashMap<String, Object>();
	    		
	    		HashMap<String, Object> imageQueryMap = new HashMap<String, Object>();
	            poseMap.put("latitude", 0f);
	    		poseMap.put("longitude", 0f);
	    		poseMap.put("altitude", (float)0.0);
	    		poseMap.put("yaw", cameraPose[0]);
	    		poseMap.put("pitch", cameraPose[1]);
	    		poseMap.put("roll", cameraPose[2]);
	    		poseMap.put("ambiguity_meters", (float)1.0e+12);
	    		pose = new JSONObject(poseMap);

	    		returnMap.put("statistics", true);
	    		returnMap.put("image_data", false);
	    		returnMap.put("estimated_client_pose", true);
	    		returnMap.put("pose_visualization_only", false);
	    		returnParams = new JSONObject(returnMap);
	    		
	    		paramsMap.put("method", "client_query");
	    		paramsMap.put("user", "test");
	    		paramsMap.put("database", "0815_db");
	    		paramsMap.put("deadline_seconds", 70.0);
	    		paramsMap.put("disable_gpu", false);
	    		paramsMap.put("perfmode", "fast");
	    		paramsMap.put("pose", pose);
	    		paramsMap.put("return", returnParams);
	    		
	    		imageQueryMap.put("params", paramsMap);
	    		imageQuery = new JSONObject(paramsMap);
	    		QueryTask qr;
	    		qr = new QueryTask(IMAGE_URL, imageQuery, "image", imgData);
	    		qr.execute(ctx); 
	    		Toast.makeText(ctx, "Sent request to server", Toast.LENGTH_SHORT).show();
	        }
	    }
	}


	public void imgLocalize(View view) {
		Context context = getBaseContext();
		Toast.makeText(context, "Taking picture", Toast.LENGTH_SHORT).show();
		writeToFile("timelog.txt", System.currentTimeMillis() + " taking picture\n");
		camera.takePicture(null, null, mPicture);
		//dispatchTakePictureIntent(1);
	}
	
	private BroadcastReceiver uiUpdated= new BroadcastReceiver() {
		
	    @Override
	    public void onReceive(Context context, Intent intent) {
	    	Date d = new Date();
	    	JSONObject loc_json = new JSONObject();
			try {
				loc_json = new JSONObject(intent.getExtras().getString("json_response"));
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				//e.printStackTrace();
			}
			
	    	if (!intent.getExtras().getString("json_response").isEmpty() )
				try {
					long t_sent = intent.getExtras().getLong("t_sent");
					long t_received = intent.getExtras().getLong("t_received");
					long t_init = intent.getExtras().getLong("t_init");
					
					double confidence = Double.parseDouble(loc_json.getString("confidence"));
					
					String loc[] = loc_json.getString("location").split("\\s");
					float new_x = Float.valueOf(loc[0]);
					float new_y = Float.valueOf(loc[1]);
					
					float old_x = new_x;
					float old_y = new_y;
					
					
					if (mParticleCloud != null) {
						String partCenter = mParticleCloud.getCenter();
			        	String[] coords;
			        	ArrayList<Step> stepHistory = new ArrayList<Step>();
			        	if (wifiService != null)
			        		stepHistory = wifiService.getStepHistory(t_init, t_received);
			        	if (stepHistory.size() > 0) {
			        		for (Step s : stepHistory) {
								new_x += s.distance*Math.sin(Math.toRadians(s.hdg));
								new_y += s.distance*Math.cos(Math.toRadians(s.hdg));
								}
			        		new_x = (old_x+new_x)/2;
			        		new_y = (old_y+new_y)/2;
			        		Point2D newCoord = mParticleCloud.getValidPoint(new Point2D(new_x,new_y), new Point2D(old_x,old_y));
			        		new_x = (float)newCoord.getX();
			        		new_y = (float)newCoord.getY();
			        		}
			        	
						/*if (intent.hasExtra("oldPosition")) {
							double[] oldCoords = intent.getExtras().getDoubleArray("oldPosition");
							Point2D newWifiPos = mParticleCloud.getShiftedCoord(new_x, new_y, oldCoords[0], oldCoords[1]);
							new_x = (float)newWifiPos.getX();
							new_y = (float)newWifiPos.getY();
							}*/
						
						String wifiResult = "Coordinates: " + new_x + " " + new_y
								+ "\nConfidence: " +loc_json.getString("confidence");
						Toast.makeText(context, wifiResult, Toast.LENGTH_SHORT).show();
						long tstamp = System.currentTimeMillis();
						String logResult = tstamp + " " + t_sent + " " + t_received + " " +old_x + " " + old_y + " -> " + new_x + " " + new_y
								+ " " +loc_json.getString("confidence") + "\n";
						
						writeToFile("loc_response.txt", logResult);
						
						writeToFile("path.txt", "w " + new_x + " "  + new_y + " " + t_init + " " + t_sent + " " + t_received +"\n");
						mParticleCloud.onRssImageUpdate(5.0, new_x, new_y, confidence,"w");
						partCenter = mParticleCloud.getCenter();
			        	coords = partCenter.split("\\s+");
			        	new_x = Float.parseFloat(coords[0]);
			        	new_y = Float.parseFloat(coords[1]);
			        	mMapView.updatePos(new_x, new_y);
						} else {
							long tstamp = System.currentTimeMillis();
							writeToFile("path.txt", "w " + new_x + " "  + new_y + " " +  tstamp + " " + 0 + " " + 0 + "\n");
							String wifiResult = "Coordinates: " + new_x + " " + new_y
									+ "\nConfidence: " +loc_json.getString("confidence");
							Toast.makeText(context, wifiResult, Toast.LENGTH_SHORT).show();
							
							String logResult = tstamp + " "  + new_x + " " + new_y
									+ " " +loc_json.getString("confidence") + "\n";
							writeToFile("loc_response.txt", logResult);
						}
					

					
					
				} catch (JSONException e) {
					//Toast.makeText(context, "No coordinates received", Toast.LENGTH_SHORT).show();
				}

	    }
	};
	
	
	private BroadcastReceiver uiUpdated_img= new BroadcastReceiver() {
		public byte[] readImgFile(String path) {
			File file = new File(path);
		    int size = (int) file.length();
		    byte[] bytes = new byte[size];
		    try {
		        BufferedInputStream buf = new BufferedInputStream(new FileInputStream(file));
		        buf.read(bytes, 0, bytes.length);
		        buf.close();
		    } catch (FileNotFoundException e) {
		        // TODO Auto-generated catch block
		        e.printStackTrace();
		    } catch (IOException e) {
		        // TODO Auto-generated catch block
		        e.printStackTrace();
		    }
		    return bytes;
		}
		
	    @Override
	    public void onReceive(Context context, Intent intent) {
	    	JSONObject loc_json = new JSONObject();
			try {
				loc_json = new JSONObject(intent.getExtras().getString("json_response"));
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				//e.printStackTrace();
			}
	    	if (!intent.getExtras().getString("json_response").isEmpty() )
				try {
					writeToFile("timelog.txt", "received " + System.currentTimeMillis() + "\n");
					
					long tstamp = System.currentTimeMillis();
					float poseYaw = Float.parseFloat(loc_json.getString("yaw"));
					float poseConfidence = Float.parseFloat(loc_json.getString("pose_confidence"));
					mImgHeading = poseYaw;
					tImgHeadingTaken = System.currentTimeMillis();
					
					String respImgPath = "";
					if (intent.getExtras().getString("imgPath") != null) {
						respImgPath = intent.getExtras().getString("imgPath");
						writeToFile("debug.txt", respImgPath);
					}
					
					
					byte[] imgbytes = readImgFile(respImgPath);
					if (respImgPath != "") {
						Bitmap bitmap = BitmapFactory.decodeByteArray(imgbytes, 0,
								imgbytes.length);
						bitmap = resizeBitmap(bitmap,400 ,400);
						mImgLocView.setImageBitmap(bitmap);
						}

					
					float new_x = Float.valueOf(loc_json.getString("local_x"));
					float new_y = Float.valueOf(loc_json.getString("local_y"));
					float retr_confidence = Float.valueOf(loc_json.getString("retrieval_confidence"));
					float img_x = new_x;
					float img_y = new_y;
					if (mParticleCloud != null) {
						
						if (imgStepHistory.size() > 0) {
							Point2D lastCoord = imgStepHistory.get(imgStepHistory.size()-1);
							Point2D firstCoord = imgStepHistory.get(0);
							
			
							Point2D imgPosition = mParticleCloud.getShiftedCoord(new_x, new_y, 
									firstCoord.getX(), firstCoord.getY());
							img_x = (float)imgPosition.getX();
							img_y = (float)imgPosition.getY();
							
							imgStepHistory.clear();
							}
						
						mParticleCloud.onRssImageUpdate(1.0-Double.parseDouble(loc_json.getString("retrieval_confidence")), img_x, img_y, (double)retr_confidence, "i");
						String partCenter = mParticleCloud.getCenter();
			        	String coords[] = partCenter.split("\\s+");
			        	new_x = Float.parseFloat(coords[0]);
			        	new_y = Float.parseFloat(coords[1]);
			        	mMapView.updatePos(new_x, new_y);
					}
					tstamp = System.currentTimeMillis();
					writeToFile("path.txt", "i " + img_x + " " + img_y + " " + tstamp + " "  + 0 + " " + 0 + "\n");
					String logResult = tstamp + " " + loc_json.getString("local_x") + " " + loc_json.getString("local_y")
							+ " " +loc_json.getString("retrieval_confidence") + " " + img_x + " "  + img_y + "\n";
					
					writeToFile("loc_response.txt", logResult);
					String imgResult = "IMG Coordinates: " + img_x + " " + img_y
							+ "\n Confidence: " +loc_json.getString("overall_confidence");
					Toast.makeText(context, imgResult, Toast.LENGTH_SHORT).show();
					
				} catch (JSONException e) {
					Toast.makeText(context, "No coordinates received", Toast.LENGTH_SHORT).show();
				}

	    }
	};
	

/*******    Camera Code     ********/
	
	
	public static void setCameraDisplayOrientation(Activity activity,
	         int cameraId, android.hardware.Camera camera) {
	     android.hardware.Camera.CameraInfo info =
	             new android.hardware.Camera.CameraInfo();
	     android.hardware.Camera.getCameraInfo(cameraId, info);
	     int rotation = activity.getWindowManager().getDefaultDisplay()
	             .getRotation();
	     int degrees = 0;
	     switch (rotation) {
	         case Surface.ROTATION_0: degrees = 0; break;
	         case Surface.ROTATION_90: degrees = 90; break;
	         case Surface.ROTATION_180: degrees = 180; break;
	         case Surface.ROTATION_270: degrees = 270; break;
	     }

	     int result;
	     if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
	         result = (info.orientation + degrees) % 360;
	         result = (360 - result) % 360;  // compensate the mirror
	     } else {  // back-facing
	         result = (info.orientation - degrees + 360) % 360;
	     }
	     camera.setDisplayOrientation(result);
	 }
	
	/** A safe way to get an instance of the Camera object. */
	public static Camera getCameraInstance(){
	    Camera c = null;
	    Log.d("CAMERA", "getting instance");
	    try {
	        c = Camera.open(); // attempt to get a Camera instance
	        Log.d("CAMERA", "success!");
	    }
	    catch (Exception e){
	        // Camera is not available (in use or does not exist)
	    }
	    return c; // returns null if camera is unavailable
	}
	
	
	
	/** A basic Camera preview class */
	public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback {
	    private SurfaceHolder mHolder;
	    private Camera mCamera;

	    public CameraPreview(Context context, Camera camera) {
	        super(context);
	        mCamera = camera;

	        // Install a SurfaceHolder.Callback so we get notified when the
	        // underlying surface is created and destroyed.
	        mHolder = getHolder();
	        mHolder.addCallback(this);
	    }

	    public void surfaceCreated(SurfaceHolder holder) {
	        // The Surface has been created, now tell the camera where to draw the preview.
	        try {
	        	//mCamera.setDisplayOrientation(90);
	            mCamera.setPreviewDisplay(holder);
	            mCamera.startPreview();
	            
	        } catch (IOException e) {
	            Log.d("TAG1: ", "Error setting camera preview: " + e.getMessage());
	        }
	    }

	    public void surfaceDestroyed(SurfaceHolder holder) {
	        // empty. Take care of releasing the Camera preview in your activity.
	    }

	    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
	        // If your preview can change or rotate, take care of those events here.
	        // Make sure to stop the preview before resizing or reformatting it.

	        if (mHolder.getSurface() == null){
	          // preview surface does not exist
	          return;
	        }

	        // stop preview before making changes
	        try {
	            mCamera.stopPreview();
	        } catch (Exception e){
	          // ignore: tried to stop a non-existent preview
	        }

	        // set preview size and make any resize, rotate or
	        // reformatting changes here

	        // start preview with new settings
	        try {
	            mCamera.setPreviewDisplay(mHolder);
	            mCamera.startPreview();

	        } catch (Exception e){
	            Log.d("TAG2: ", "Error starting camera preview: " + e.getMessage());
	        }
	    }
	}
	
 /* CPU usage */	
	private float readUsage() {
	    try {
	        RandomAccessFile reader = new RandomAccessFile("/proc/stat", "r");
	        String load = reader.readLine();

	        String[] toks = load.split(" ");

	        long idle1 = Long.parseLong(toks[5]);
	        long cpu1 = Long.parseLong(toks[2]) + Long.parseLong(toks[3]) + Long.parseLong(toks[4])
	              + Long.parseLong(toks[6]) + Long.parseLong(toks[7]) + Long.parseLong(toks[8]);

	        try {
	            Thread.sleep(360);
	        } catch (Exception e) {}

	        reader.seek(0);
	        load = reader.readLine();
	        reader.close();

	        toks = load.split(" ");

	        long idle2 = Long.parseLong(toks[5]);
	        long cpu2 = Long.parseLong(toks[2]) + Long.parseLong(toks[3]) + Long.parseLong(toks[4])
	            + Long.parseLong(toks[6]) + Long.parseLong(toks[7]) + Long.parseLong(toks[8]);

	        return (float)(cpu2 - cpu1) / ((cpu2 + idle2) - (cpu1 + idle1));

	    } catch (IOException ex) {
	        ex.printStackTrace();
	    }

	    return 0;
	} 

}


