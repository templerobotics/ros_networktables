#!/usr/bin/env python3
import rospy
from ros_networktables.msg import NTEntryUpdate, TXArrayMessage, TXMessage
from ros_networktables.srv import GetNTEntry, GetNTEntryArray, SetNTEntry, SetNTEntryArray
from networktables import NetworkTables


class NTbridge(object):
    def __init__(self):
        NetworkTables.initialize(server=rospy.get_param('/nt_bridge/server', '10.12.34.2')) # VMX-pi IP Address
        self._tables = {}

        rx_tables: list[str] = rospy.get_param('/nt_bridge/tables/rx', [])
        tx_tables: list[str] = rospy.get_param('/nt_bridge/tables/tx', [])

        for table in rx_tables + tx_tables:
            entry = {'networktable': NetworkTables.getTable(table)}
            if table in rx_tables:
                entry['rx_publisher'] = rospy.Publisher(f'/nt_bridge/{table}/rx', NTEntryUpdate, queue_size=10)
                entry['networktable'].addEntryListener(self._table_update, localNotify=rospy.get_param('/nt_bridge/localnotify', False))
            if table in tx_tables:
                entry['tx_subscriber'] = rospy.Subscriber(f'/nt_bridge/{table}/tx', TXMessage, self._table_tx, callback_args=table)
                entry['tx_array_subscriber'] = rospy.Subscriber(f'/nt_bridge/{table}/tx_array', TXArrayMessage, self._table_tx_array, callback_args=table) 
            self._tables[table] = entry
        
        self.get_entry_service = rospy.Service('/nt_bridge/get_entry', GetNTEntry, self._get_entry_string)
        self.get_entry_arr_service = rospy.Service('/nt_bridge/get_entry_array', GetNTEntryArray, self._get_entry_string_array)
        self.set_entry_service = rospy.Service('/nt_bridge/set_entry', SetNTEntry, self._set_entry_string)
        self.set_entry_arr_service = rospy.Service('/nt_bridge/set_entry_array', SetNTEntryArray, self._set_entry_string_array)

    def _table_update(self, table, key, value, isNew):
        ftable = str(table).split('/')[1]
        # rospy.loginfo(f"Table Update Triggered. Table: {ftable} Key: {key} Value: {value} IsNew: {isNew}")
        entry = NTEntryUpdate(table=ftable, key=key, type=str(type(value)).split("'")[1], value=str(value), time=rospy.get_rostime())
        self._tables[ftable]['rx_publisher'].publish(entry)
    
    def _table_tx(self, data, table: str):
        # rospy.loginfo(f"Table TX {table}: {data}")
        self._tables[table]['networktable'].getEntry(data.key).setString(str(data.value))

    def _table_tx_array(self, data, table: str):
        # rospy.loginfo(f"Table Array TX {table}: {data}")
        self._tables[table]['networktable'].getEntry(data.key).setStringArray(data.value)

    def _get_entry_string(self, req):
        return self._tables[req.table]['networktable'].getEntry(req.key).getString(None)

    def _get_entry_string_array(self, req):
        return self._tables[req.table]['networktable'].getEntry(req.key).getStringArray(None)

    def _set_entry_string(self, req):
        return self._tables[req.table]['networktable'].getEntry(req.key).setString(req.value)

    def _set_entry_string_array(self, req):
        return self._tables[req.table]['networktable'].getEntry(req.key).setStringArray(req.value)


if __name__ == '__main__':
    rospy.init_node('nt_bridge')
    rospy.loginfo('Starting NetworkTables bridge')
    ntbridge = NTbridge()
    rospy.spin()
