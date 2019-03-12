import gnu.io.*;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Enumeration;

public class SerialPort_Manager implements SerialPortEventListener {


    /*
     * -------------标志位声明——————————
     * @MoveIsFinish：代表上一个动作完成
     * @SavePosition:代表接下来的是保存操作
     * @ZeroPosition;表示开机零位是否取到
     * @AutoMove：自动指向使能（确定）
     * @SerialPortIsOpen：串口是否已经打开，避免重复打开串口
     * @DataAvalible：串口有数据标志位
     * @serialPort：当前的串口
     * @Moving：下一个动作指令的缓存
     * @Angel：六轴角度的缓存
     */
    private String[] Moving = null;
    private String[] Angel = null;

    private boolean MoveIsFinish = true;
    private boolean SavePosition = false;
    private boolean ZeroPosition = false;
    private boolean AutoMove = false;
    private boolean SerialPortIsOpen = false, DataAvalible = false; //程序运行的各个标志位
    private SerialPort serialPort;        //打开的串口
    private final int baudrate = 115200;

    public void setZeroPosition(boolean zeroPosition) {
        ZeroPosition = zeroPosition;
    }

    public boolean isMoveIsFinish() {
        return MoveIsFinish;
    }

    public void setMoveIsFinish(boolean moveIsFinish) {
        MoveIsFinish = moveIsFinish;
    }

    public boolean isSavePosition() {
        return SavePosition;
    }

    public void setSavePosition(boolean savePosition) {
        SavePosition = savePosition;
    }

    public boolean isSerialPortIsOpen() {
        return SerialPortIsOpen;
    }

    public void setSerialPortIsOpen(boolean serialPortIsOpen) {
        SerialPortIsOpen = serialPortIsOpen;
    }

    public boolean isDataAvalible() {
        return DataAvalible;
    }

    public void setDataAvalible(boolean dataAvalible) {
        DataAvalible = dataAvalible;
    }

    public String[] getMoving() {
        return Moving;
    }

    public void setMoving(String[] moving) {
        Moving = moving;
    }

    public SerialPort getSerialPort() {
        return serialPort;
    }

    public boolean isZeroPosition() {
        return ZeroPosition;
    }

    public boolean isAutoMove() {
        return AutoMove;
    }

    public void setAutoMove(boolean autoMove) {
        AutoMove = autoMove;
    }

    public String[] getAngel() {
        return Angel;
    }

    /**
     * @return portNameList 可用端口列表
     */
    public final ArrayList<String> findPort() {
        // 获得当前所有可用串口
        Enumeration<CommPortIdentifier> portList = CommPortIdentifier.getPortIdentifiers();
        ArrayList<String> portNameList = new ArrayList<>();
        // 将可用串口名添加到List并返回该List
        while (portList.hasMoreElements()) {
            String portName = portList.nextElement().getName();
            portNameList.add(portName);
        }
        return portNameList;
    }

    /*
     * @param portName 端口名称
     *
     * @param baudrate 波特率
     *
     * @return SerialPort 对象
     *
     * */
    public final void openPort(String portName, int bandrate) {
        if (!SerialPortIsOpen) {
            try {
                //识别端口
                CommPortIdentifier portIdntifier = CommPortIdentifier.getPortIdentifier(portName);
                CommPort commPort = portIdntifier.open(portName, 2000);
                SerialPortIsOpen = true;
                //打开端口并设置超时响应时间
                if (commPort instanceof SerialPort) {
                    serialPort = (SerialPort) commPort;
                    try {
                        addListener(serialPort, this);
                        //设置串口参数
                        serialPort.setSerialPortParams(bandrate,
                                serialPort.DATABITS_8,
                                serialPort.STOPBITS_1,
                                serialPort.PARITY_NONE);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                } else {
                    throw new PortInUseException();
                }


            } catch (Exception e) {

                e.printStackTrace();
            }
        } else if (SerialPortIsOpen) {
            SerialPortIsOpen = false;
            closePort(serialPort);

        }
    }

    private static void closePort(SerialPort serialPort) {
        if (serialPort != null) {
            serialPort.close();
        }
    }

    /**
     * 向串口发送数据
     *
     * @param serialPort 串口对象
     * @param data       待发送数据
     */
    public void sendToPort(SerialPort serialPort, byte[] data) {
        OutputStream out = null;
        try {
            out = serialPort.getOutputStream();
            try {
                serialPort.setSerialPortParams(baudrate,
                        serialPort.DATABITS_8,
                        serialPort.STOPBITS_1,
                        serialPort.PARITY_NONE);
            } catch (Exception e) {
                e.printStackTrace();
            }
            try {
                out.write(data);
                out.write("\r\n".getBytes());
                out.flush();
            } catch (Exception e) {
                e.printStackTrace();
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
                if (out != null) {
                    out.close();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * 从串口读取数据
     *
     * @param serialPort 当前已建立连接的SerialPort对象
     * @return 读取到的数据
     */
    private static byte[] readFromPort(SerialPort serialPort) {

        InputStream in = null;
        byte[] bytes = null;
        try {
            in = serialPort.getInputStream();
            //获取buffer里的数据长度
            int bufLength = in.available();
            while (bufLength != 0) {
                bytes = new byte[bufLength];
                in.read(bytes);
                bufLength = in.available();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                if (in != null) {
                    in.close();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return bytes;
    }

    /**
     * 添加监听器
     *
     * @param serialPort, 串口对象
     * @param listener    串口监听器
     */
    private static void addListener(SerialPort serialPort, SerialPortEventListener listener) {
        try {
            serialPort.addEventListener(listener);
            serialPort.notifyOnDataAvailable(true);
            serialPort.notifyOnBreakInterrupt(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    @Override
    public void serialEvent(SerialPortEvent event) {
        // TODO Auto-generated method stub
        try {
            if (event.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
                byte[] bytes = readFromPort(serialPort);
                String str = new String(bytes);
                if (str.startsWith("ok")) {
                     System.out.println(str);
                    Angel = str.split(",");
                    if (!ZeroPosition) {
                        ZeroPosition = true;
                    } else {
                        SavePosition = true;
                        DataAvalible = true;
                    }
                } else if (str.startsWith("mode")) {
                    System.out.println(str);
                    DataAvalible = true;
                    Moving = str.split(",");
                    MoveIsFinish = true;
                } else if (str.startsWith("point")) {
                    AutoMove = true;
                    DataAvalible = true;
                }

            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

