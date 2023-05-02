package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {
  HashMap<String, Integer> m_identificationTable = new HashMap<String, Integer>();
  ArrayList<StringLogEntry> m_StringLogEntries = new ArrayList<StringLogEntry>();
  ArrayList<BooleanLogEntry> m_BooleanLogEntries = new ArrayList<BooleanLogEntry>();
  ArrayList<DoubleLogEntry> m_DoubleLogEntries = new ArrayList<DoubleLogEntry>();

  DataLog m_DataLog = DataLogManager.getLog();

  public void addStringLogEntry(String name) {
    // Update identification pairing
    m_identificationTable.put(name, m_StringLogEntries.size());

    // Add to internal entry list
    StringLogEntry newEntry = new StringLogEntry(m_DataLog, name);
    m_StringLogEntries.add(newEntry);
  }

  public void addBooleanLogEntry(String name) {
    // Update identification pairing
    m_identificationTable.put(name, m_BooleanLogEntries.size());

    // Add to internal entry list
    BooleanLogEntry newEntry = new BooleanLogEntry(m_DataLog, name);
    m_BooleanLogEntries.add(newEntry);
  }

  public void addDoubleLogEntry(String name) {
    // Update identification pairing
    m_identificationTable.put(name, m_DoubleLogEntries.size());

    // Add to internal entry list
    DoubleLogEntry newEntry = new DoubleLogEntry(m_DataLog, name);
    m_DoubleLogEntries.add(newEntry);
  }

  public void append(String name, String value) {
    if (m_identificationTable.containsKey(name)) {
      int index = m_identificationTable.get(name); // Get index from identification table
      m_StringLogEntries.get(index).append(value); // Append the new value
    }
  }

  public void append(String name, Boolean value) {
    if (m_identificationTable.containsKey(name)) { 
      int index = m_identificationTable.get(name); // Get index from identification table
      m_BooleanLogEntries.get(index).append(value); // Append the new value
    }
  }

  public void append(String name, Double value) {
    if (m_identificationTable.containsKey(name)) {
      int index = m_identificationTable.get(name); // Get index from identification table
      m_DoubleLogEntries.get(index).append(value); // Append the new value
    }
  }

  public void log(String message) {
    DataLogManager.log(message);
  }
}