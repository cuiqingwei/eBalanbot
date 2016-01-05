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

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.ToggleButton;

import com.actionbarsherlock.app.SherlockFragment;

public class InfoFragment extends SherlockFragment {
    static TextView mAppVersion, mFirmwareVersion, mEEPROMVersion, mMcu, mBatteryLevel, mRuntime;
    static ToggleButton mToggleButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.info, container, false);
        mAppVersion = (TextView) v.findViewById(R.id.appVersion);
        mFirmwareVersion = (TextView) v.findViewById(R.id.firmwareVersion);
        mEEPROMVersion = (TextView) v.findViewById(R.id.eepromVersion);
        mMcu = (TextView) v.findViewById(R.id.mcu);
        mBatteryLevel = (TextView) v.findViewById(R.id.batterylevel);
        mRuntime = (TextView) v.findViewById(R.id.runtime);

        mToggleButton = (ToggleButton) v.findViewById(R.id.button);
        mToggleButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (((ToggleButton) v).isChecked())
                    mToggleButton.setText("Stop");
                else
                    mToggleButton.setText("Start");

                if (eBalanbotActivity.mChatService != null) {
                    if (eBalanbotActivity.mChatService.getState() == BluetoothChatService.STATE_CONNECTED && eBalanbotActivity.checkTab(ViewPagerAdapter.INFO_FRAGMENT)) {
                        if (((ToggleButton) v).isChecked())
                            eBalanbotActivity.mChatService.write(eBalanbotActivity.statusBegin); // Request data
                        else
                            eBalanbotActivity.mChatService.write(eBalanbotActivity.statusStop); // Stop sending data
                    }
                }
            }
        });

        if (eBalanbotActivity.mChatService != null) {
            if (eBalanbotActivity.mChatService.getState() == BluetoothChatService.STATE_CONNECTED && eBalanbotActivity.checkTab(ViewPagerAdapter.INFO_FRAGMENT)) {
                if (mToggleButton.isChecked())
                    eBalanbotActivity.mChatService.write(eBalanbotActivity.statusBegin); // Request data
                else
                    eBalanbotActivity.mChatService.write(eBalanbotActivity.statusStop); // Stop sending data
            }
        }

        updateView();
        return v;
    }

    public static void updateView() {
        if (mAppVersion != null && eBalanbotActivity.appVersion != null)
            mAppVersion.setText(eBalanbotActivity.appVersion);
        if (mFirmwareVersion != null && eBalanbotActivity.firmwareVersion != null)
            mFirmwareVersion.setText(eBalanbotActivity.firmwareVersion);
        if (mEEPROMVersion != null && eBalanbotActivity.eepromVersion != null)
            mEEPROMVersion.setText(eBalanbotActivity.eepromVersion);
        if (mMcu != null && eBalanbotActivity.mcu != null)
            mMcu.setText(eBalanbotActivity.mcu);
        if (mBatteryLevel != null && eBalanbotActivity.batteryLevel != null)
            mBatteryLevel.setText(eBalanbotActivity.batteryLevel + 'V');
        if (mRuntime != null && eBalanbotActivity.runtime != 0) {
            String minutes = Integer.toString((int) Math.floor(eBalanbotActivity.runtime));
            String seconds = Integer.toString((int) (eBalanbotActivity.runtime % 1 / (1.0 / 60.0)));
            mRuntime.setText(minutes + " min " + seconds + " sec");
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        updateView(); // When the user resumes the view, then update the values

        if (mToggleButton.isChecked())
            mToggleButton.setText("Stop");
        else
            mToggleButton.setText("Start");

        if (eBalanbotActivity.mChatService != null) {
            if (eBalanbotActivity.mChatService.getState() == BluetoothChatService.STATE_CONNECTED && eBalanbotActivity.checkTab(ViewPagerAdapter.INFO_FRAGMENT)) {
                if (mToggleButton.isChecked())
                    eBalanbotActivity.mChatService.write(eBalanbotActivity.statusBegin); // Request data
                else
                    eBalanbotActivity.mChatService.write(eBalanbotActivity.statusStop); // Stop sending data
            }
        }
    }
}