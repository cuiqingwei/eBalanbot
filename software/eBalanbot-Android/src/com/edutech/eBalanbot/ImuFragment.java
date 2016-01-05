/*************************************************************************************
 * Copyright (C) 2012-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.
 *
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL2) as published by the Free Software
 * Foundation and appearing in the file GPL2.TXT included in the packaging of
 * this file. Please note that GPL2 Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of
 * the GPL2 ("Copyleft").
 *
 * Contact information
 * -------------------
 *
 * Kristian Lauszus, TKJ Electronics
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  kristianl@tkjelectronics.com
 *
 ************************************************************************************/

package com.edutech.eBalanbot;

import android.annotation.TargetApi;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TableRow;
import android.widget.TextView;

import com.actionbarsherlock.app.SherlockFragment;
import com.edutech.eBalanbot.R;

public class ImuFragment extends SherlockFragment {
    private Button mButton;
    public TextView mPitchView;
    public TextView mRollView;
    public TextView mCoefficient;
    private TableRow mTableRow;

    private Handler mHandler = new Handler();
    private Runnable mRunnable;
    private int counter = 0;
    boolean buttonState;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.imu, container, false);

        mPitchView = (TextView) v.findViewById(R.id.textView1);
        mRollView = (TextView) v.findViewById(R.id.textView2);
        mCoefficient = (TextView) v.findViewById(R.id.textView3);
        mTableRow = (TableRow) v.findViewById(R.id.tableRowCoefficient);
        mButton = (Button) v.findViewById(R.id.button);

        mHandler.postDelayed(new Runnable() { // Hide the menu icon and tablerow if there is no build in gyroscope in the device
            @Override
            public void run() {
                if (SensorFusion.IMUOutputSelection == -1)
                    mHandler.postDelayed(this, 100); // Run this again if it hasn't initialized the sensors yet
                else if (SensorFusion.IMUOutputSelection != 2) // Check if a gyro is supported
                    mTableRow.setVisibility(View.GONE); // If not then hide the tablerow
            }
        }, 100); // Wait 100ms before running the code

        eBalanbotActivity.buttonState = false;

        return v;
    }

    @Override
    public void onResume() {
        super.onResume();

        mRunnable = new Runnable() {
            @Override
            public void run() {
                mHandler.postDelayed(this, 50); // Update IMU data every 50ms
                if (eBalanbotActivity.mSensorFusion == null)
                    return;
                mPitchView.setText(eBalanbotActivity.mSensorFusion.pitch);
                mRollView.setText(eBalanbotActivity.mSensorFusion.roll);
                mCoefficient.setText(eBalanbotActivity.mSensorFusion.coefficient);

                counter++;
                if (counter > 2) { // Only send data every 150ms time
                    counter = 0;
                    if (eBalanbotActivity.mChatService == null)
                        return;
                    if (eBalanbotActivity.mChatService.getState() == BluetoothChatService.STATE_CONNECTED && eBalanbotActivity.currentTabSelected == ViewPagerAdapter.IMU_FRAGMENT) {
                        buttonState = mButton.isPressed();
                        eBalanbotActivity.buttonState = buttonState;

                        if (eBalanbotActivity.joystickReleased || getResources().getConfiguration().orientation != Configuration.ORIENTATION_LANDSCAPE) // Check if joystick is released or we are not in landscape mode
                            CustomViewPager.setPagingEnabled(!buttonState); // Set the ViewPager according to the button
                        else
                            CustomViewPager.setPagingEnabled(false);

                        if (eBalanbotActivity.joystickReleased) {
                            if (buttonState) {
                                lockRotation();
                                eBalanbotActivity.mChatService.write(eBalanbotActivity.sendIMUValues + eBalanbotActivity.mSensorFusion.pitch + ',' + eBalanbotActivity.mSensorFusion.roll + ";");
                                mButton.setText(R.string.sendingData);
                            } else {
                                unlockRotation();
                                eBalanbotActivity.mChatService.write(eBalanbotActivity.sendStop);
                                mButton.setText(R.string.notSendingData);
                            }
                        }
                    } else {
                        mButton.setText(R.string.button);
                        if (eBalanbotActivity.currentTabSelected == ViewPagerAdapter.IMU_FRAGMENT && eBalanbotActivity.joystickReleased)
                            CustomViewPager.setPagingEnabled(true);
                    }
                }
            }
        };
        mHandler.postDelayed(mRunnable, 50); // Update IMU data every 50ms
    }

    @TargetApi(Build.VERSION_CODES.JELLY_BEAN_MR2)
    private void lockRotation() {
        if (getResources().getBoolean(R.bool.isTablet)) { // Check if the layout can rotate
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2)
                getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LOCKED); // Lock screen orientation so it doesn't rotate
            else { // Lock rotation manually - source: http://blogs.captechconsulting.com/blog/eric-miles/programmatically-locking-android-screen-orientation
                int rotation = eBalanbotActivity.getRotation();
                int lock;

                if (rotation == Surface.ROTATION_90) // Landscape
                    lock = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE;
                else if (rotation == Surface.ROTATION_180) // Reverse Portrait
                    lock = ActivityInfo.SCREEN_ORIENTATION_REVERSE_PORTRAIT;
                else if (rotation == Surface.ROTATION_270) // Reverse Landscape
                    lock = ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE;
                else
                    lock = ActivityInfo.SCREEN_ORIENTATION_PORTRAIT;
                getActivity().setRequestedOrientation(lock);
            }
        }
    }

    @TargetApi(Build.VERSION_CODES.JELLY_BEAN_MR2)
    private void unlockRotation() {
        if (getResources().getBoolean(R.bool.isTablet)) { // Check if the layout can rotate
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2)
                getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_FULL_USER); // Unlock screen orientation
            else
                getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_FULL_SENSOR); // Unlock screen orientation
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        mHandler.removeCallbacks(mRunnable);
        unlockRotation();
    }
}