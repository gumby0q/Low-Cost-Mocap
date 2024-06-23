import React, { useState, useEffect, useRef } from 'react';
import './LogUI.css';
import { socket } from '../shared/styles/scripts/socket';

const LogUI = () => {
    const [logs, setLogs] = useState([
        // { id: 200, status: 'ok', message: 'System started successfully.' },
        // { id: 201, status: 'ok', message: 'Connected to database.' },
        // { id: 202, status: 'error', message: 'Failed to retrieve data from API.' },
        // { id: 203, status: 'ok', message: 'User logged in.' },
        // { id: 204, status: 'error', message: 'File not found.' }
    ]);   
    
    const [crcErrors, setCrcErrors] = useState(0);

    const maxLogs = 10;
    const MAX_DATA_LENGTH = maxLogs;
    const logContentRef = useRef(null);

    // useEffect(() => {
    //     if (logs.length > maxLogs) {
    //         setLogs(logs.slice(logs.length - maxLogs));
    //     }
    // }, [logs]);

    // const addLog = (log) => {
    //     setLogs([...logs, log]);
    // };

    const clearLogs = () => {
        setLogs([]);
    };

    useEffect(() => {
        const listener = (data) => {
            const type = data["type"];
            const data_str = data["data"];

            //   console.log("data", data);

            if (data === "Invalid checksum on device") {
                setCrcErrors(num => {
                    return num + 1;
                })
            } else {
                setLogs((prevData) => {
                    const updatedLabels = [...prevData,
                    {
                        type: type,
                        time: new Date().toLocaleTimeString(),
                        message: data_str,
                    }
                    ];
    
                    if (updatedLabels.length > MAX_DATA_LENGTH) {
                        updatedLabels.shift();
                    }
    
                    return updatedLabels;
                });
            }
        }
        socket.on("serial-port-log", listener)

        return () => {
            socket.off("serial-port-log", listener)
        }
    }, [setLogs, setCrcErrors])


    return (
        <div className="log-container">
            <div className="log-header">
                <h4>System Logs</h4>
                <button onClick={clearLogs}>Clear Logs</button>
            </div>
            <p>Check if PID and TRIM settings logs is OK!</p>
            <div className="log-content" ref={logContentRef}>
                {logs.map((log, index) => {
                        const logStyle = log.message.includes("ERROR") ? 'error' : 'ok';
                        return <div key={index} className={`log-entry ${logStyle}`}>
                            {/* [{log.id}] {log.message} */}
                            [{log.time}] {log.type}: {log.message}
                        </div>
                    })
                }
            </div>
            <p>Invalid checksum on device number: {crcErrors}</p>
            {/* <button onClick={() => addLog({ time: new Date().toLocaleTimeString(), id: 205, status: 'ok', message: 'New log entry.' })}>Add Log</button> */}
        </div>
    );
};

export default LogUI;
