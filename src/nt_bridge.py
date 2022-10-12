#!/usr/bin/env python3
import rospy
from ros_networktables.msg import NTEntryUpdate, TXArrayMessage, TXMessage
from ros_networktables.srv import *
from networktables import NetworkTables, NetworkTable
from dataclasses import dataclass

class NTbridge(object):

    @dataclass(init=False)
    class TableEntry(object):
        networktable: NetworkTable
        rx_publisher: rospy.Publisher
        tx_subscriber: rospy.Subscriber
        tx_array_subscriber: rospy.Subscriber
            

    def __init__(self) -> None:
        NetworkTables.initialize(server=rospy.get_param('~server', '10.12.34.2')) # VMX-pi IP Address
        self._tables: dict[str, type[self.TableEntry]] = {}

        rx_tables: set[str] = set(rospy.get_param('~tables/rx', []))
        tx_tables: set[str] = set(rospy.get_param('~tables/tx', []))

        for table in rx_tables.union(tx_tables): # Ensures table only loops once if it exists in both sets
            entry = self.TableEntry()
            entry.networktable = NetworkTables.getTable(table)
            if table in rx_tables:
                entry.rx_publisher = rospy.Publisher(f'~{table}/rx', NTEntryUpdate, queue_size=10)
                entry.networktable.addEntryListener(self._table_update, localNotify=rospy.get_param('~localnotify', False))
            if table in tx_tables:
                entry.tx_subscriber = rospy.Subscriber(f'~{table}/tx', TXMessage, self._table_tx, callback_args=table)
                entry.tx_array_subscriber = rospy.Subscriber(f'~{table}/tx_array', TXArrayMessage, self._table_tx_array, callback_args=table) 
            self._tables[table] = entry
        
        self._get_entry_service = rospy.Service('~get_entry', GetNTEntry, self._get_entry_string)
        self._get_entry_arr_service = rospy.Service('~get_entry_array', GetNTEntryArray, self._get_entry_string_array)
        self._set_entry_service = rospy.Service('~set_entry', SetNTEntry, self._set_entry_string)
        self._set_entry_arr_service = rospy.Service('~set_entry_array', SetNTEntryArray, self._set_entry_string_array)

    def _tx_check(self) -> None:
        if (not NetworkTables.isConnected()):
            rospy.logerr("NetworkTables bridge is not connected to a NetworkTables instance.")
    
    def _table_update(self, table, key, value, isNew) -> None:
        ftable = str(table).split('/')[1]
        # rospy.loginfo(f"Table Update Triggered. Table: {ftable} Key: {key} Value: {value} IsNew: {isNew}")
        entry = NTEntryUpdate(table=ftable, key=key, type=str(type(value)).split("'")[1], value=str(value), time=rospy.get_rostime())
        self._tables[ftable].rx_publisher.publish(entry)
    
    def _table_tx(self, data, table: str) -> None:
        # rospy.loginfo(f"Table TX {table}: {data}")
        self._tx_check()
        self._tables[table].networktable.getEntry(data.key).setString(str(data.value))

    def _table_tx_array(self, data, table: str) -> None:
        # rospy.loginfo(f"Table Array TX {table}: {data}")
        self._tx_check()
        self._tables[table].networktable.getEntry(data.key).setStringArray(data.value)

    def _get_entry_string(self, req: GetNTEntryRequest) -> GetNTEntryResponse:
        return GetNTEntryResponse(self._tables[req.table].networktable.getEntry(req.key).getString(None))

    def _get_entry_string_array(self, req: GetNTEntryArrayRequest) -> GetNTEntryArrayResponse:
        return GetNTEntryArrayResponse(self._tables[req.table].networktable.getEntry(req.key).getStringArray(None))

    def _set_entry_string(self, req: SetNTEntryRequest) -> SetNTEntryResponse:
        return SetNTEntryResponse(self._tables[req.table].networktable.getEntry(req.key).setString(req.value))

    def _set_entry_string_array(self, req: SetNTEntryArrayRequest) -> SetNTEntryArrayResponse:
        return SetNTEntryArrayResponse(self._tables[req.table].networktable.getEntry(req.key).setStringArray(req.value))


if __name__ == '__main__':
    rospy.init_node('nt_bridge')
    rospy.loginfo('Starting NetworkTables bridge')
    ntbridge = NTbridge()
    rospy.spin()
