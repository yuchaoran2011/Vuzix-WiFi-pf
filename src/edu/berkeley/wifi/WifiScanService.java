package edu.berkeley.wifi;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

import org.json.JSONObject;

import pf.utils.Point2D;
import android.app.Activity;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Binder;
import android.os.IBinder;
import android.net.Uri;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Message;
import android.os.Messenger;
import android.util.Log;
import android.widget.Toast;
import android.content.BroadcastReceiver;

class ScanComparable implements Comparator<ScanResult> {
	 
    @Override
    public int compare(ScanResult s1, ScanResult s2) {
        return (s1.level>s2.level ? -1 : (s1.level==s2.level ? 0 : 1));
    }
}

public class WifiScanService extends Service {
	private final IBinder mBinder = new MyBinder();
	WifiManager wifi;
	Integer floor = 1;
	String cutoff_freq = "7000";
	Integer requestId = 1;
	static final String WIFI_URL = "http://sofia.eecs.berkeley.edu:8003/wifi/submit_fingerprint";
	Point2D wifiPos;
	ArrayList<Point2D> cloudPos;
	ArrayList<Step> stepHistory;
	HashMap<Integer,long[]> wifiRequests;
	public void writeToFile(String data)  {
		File root = new File(Environment.getExternalStorageDirectory()+File.separator+"wifiloc");
	   
	    File file = new File(root, "wifiscan.txt");
	    FileWriter filewriter;
		try {
			filewriter = new FileWriter(file,true);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			Log.d("WIFIRES","Could not create file " + e1.getMessage());
			return;
		}
	     
	    BufferedWriter out = new BufferedWriter(filewriter);
			try {
			out.write(data);
			out.close();
			} catch (IOException e)
			{
			Log.d("WIFIRES","Could not write to file " + e.getMessage());
			}
	}
	
	private final BroadcastReceiver receiverWifi = new BroadcastReceiver(){
		  
		  	
			@Override
		    public void onReceive(Context context, Intent intent) {
		        List<ScanResult> scanResults = wifi.getScanResults();
				
				if(scanResults == null || scanResults.isEmpty()) {
					Log.d("WIFI_RECEIVER","No APs detected :(");
					Toast.makeText(context, "No APs detected :(", Toast.LENGTH_SHORT).show();
				} else {
					Collections.sort(scanResults, new ScanComparable());
					HashMap<String,Integer> reqParams = new HashMap<String,Integer>();
					String logResult = "";
					long tstamp = System.currentTimeMillis();
					for (ScanResult scan : scanResults) {
						//int linearLevel = WifiManager.calculateSignalLevel(scan.level, 99);
						//macRSSI.put(scan.BSSID.toString(), scan.level*100-linearLevel);
						reqParams.put(scan.BSSID.toString(), scan.level);
						logResult += tstamp + " " + scan.BSSID.toString() + " " + scan.level + " "+  scan.frequency + " " + scan.SSID + "\n";
					}
					writeToFile(logResult);
					reqParams.put("cluster_id", floor);
					reqParams.put("request_id", requestId);
					//reqParams.put("freq_filter", Integer.parseInt(cutoff_freq));
					JSONObject queryCore = new JSONObject(reqParams);
					HashMap<String, JSONObject> postedData = new HashMap<String, JSONObject>();
					postedData.put("fingerprint_data", queryCore);
					
					JSONObject query = new JSONObject(postedData);
					
					QueryTask qr;
					/*if (cloudPos.size() > 0) {
						qr = new QueryTask(WIFI_URL, query, "wifi", cloudPos.get(0));
						cloudPos.clear();
						}*/
					//else
					if (!wifiRequests.containsKey(requestId))
						Log.d("REQMIS", "ALERT: " + requestId + " " + wifiRequests.size());
					else {
						long[] ts = wifiRequests.get(requestId);
						ts[1] = System.currentTimeMillis();
						wifiRequests.put(requestId, ts);
						qr = new QueryTask(WIFI_URL, query, "wifi", wifiRequests);
						qr.execute(context);
						
						requestId++;
						}
				}
		        //Toast.makeText(context, "Scan complete.", Toast.LENGTH_SHORT).show();
		    }   
		};

	@Override 
	public void onCreate() {
		IntentFilter filter = new IntentFilter();
		filter.addAction("android.net.wifi.SCAN_RESULTS");
		registerReceiver(receiverWifi, filter);
		cloudPos = new ArrayList<Point2D>();
		stepHistory = new ArrayList<Step>();
		wifiRequests = new HashMap<Integer,long[]>();
	}
	
	@Override
	public void onDestroy() {
		super.onDestroy();
		unregisterReceiver(receiverWifi);
	}
	@Override
	  public int onStartCommand(Intent intent, int flags, int startId) {
	    Log.d("WIFISCAN","service started!");
	    Bundle b = intent.getExtras();
		floor = b.getInt("floor_id");
		cutoff_freq = b.getString("cutoff_freq");
		
		long[] ts = new long[2];
		ts[0] = System.currentTimeMillis();
		wifiRequests.put(requestId, ts);
		wifi = (WifiManager)getSystemService(Context.WIFI_SERVICE);
		if (!wifi.startScan()) {
	    	Log.d("SCANNING_FAILURE","Wifi is turned off!");
	    	}
		return Service.START_NOT_STICKY;
	  }
	
	public void setCloudPosition(Point2D newPos) {
		cloudPos.add(newPos);
	}
	public void addStep(Step s) {
		stepHistory.add(s);
	}
	
	public ArrayList<Step> getStepHistory(long ts_from, long ts_to) {
		ArrayList<Step> stepHist = new ArrayList<Step>();
		
		for (Step s: stepHistory) {
			if (s.tstamp > ts_to)
				break;
			if (s.tstamp >= ts_from)
				stepHist.add(s);
			}
		Log.d("STEPHIST", "STEPS: "+ stepHist.size());
		return stepHist;
	}

	  @Override
	  public IBinder onBind(Intent intent) {
	  //TODO for communication return IBinder implementation
	    return mBinder;
	  }
	  
	  public class MyBinder extends Binder {
		    WifiScanService getService() {
		      return WifiScanService.this;
		    }
		  }
	  


}

