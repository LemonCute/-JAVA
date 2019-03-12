import gnu.io.*;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Enumeration;

public class SerialPort_Manager implements SerialPortEventListener {


    /*
     * -------------��־λ������������������������
     * @MoveIsFinish��������һ���������
     * @SavePosition:������������Ǳ������
     * @ZeroPosition;��ʾ������λ�Ƿ�ȡ��
     * @AutoMove���Զ�ָ��ʹ�ܣ�ȷ����
     * @SerialPortIsOpen�������Ƿ��Ѿ��򿪣������ظ��򿪴���
     * @DataAvalible�����������ݱ�־λ
     * @serialPort����ǰ�Ĵ���
     * @Moving����һ������ָ��Ļ���
     * @Angel������ǶȵĻ���
     */
    private String[] Moving = null;
    private String[] Angel = null;

    private boolean MoveIsFinish = true;
    private boolean SavePosition = false;
    private boolean ZeroPosition = false;
    private boolean AutoMove = false;
    private boolean SerialPortIsOpen = false, DataAvalible = false; //�������еĸ�����־λ
    private SerialPort serialPort;        //�򿪵Ĵ���
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
     * @return portNameList ���ö˿��б�
     */
    public final ArrayList<String> findPort() {
        // ��õ�ǰ���п��ô���
        Enumeration<CommPortIdentifier> portList = CommPortIdentifier.getPortIdentifiers();
        ArrayList<String> portNameList = new ArrayList<>();
        // �����ô�������ӵ�List�����ظ�List
        while (portList.hasMoreElements()) {
            String portName = portList.nextElement().getName();
            portNameList.add(portName);
        }
        return portNameList;
    }

    /*
     * @param portName �˿�����
     *
     * @param baudrate ������
     *
     * @return SerialPort ����
     *
     * */
    public final void openPort(String portName, int bandrate) {
        if (!SerialPortIsOpen) {
            try {
                //ʶ��˿�
                CommPortIdentifier portIdntifier = CommPortIdentifier.getPortIdentifier(portName);
                CommPort commPort = portIdntifier.open(portName, 2000);
                SerialPortIsOpen = true;
                //�򿪶˿ڲ����ó�ʱ��Ӧʱ��
                if (commPort instanceof SerialPort) {
                    serialPort = (SerialPort) commPort;
                    try {
                        addListener(serialPort, this);
                        //���ô��ڲ���
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
     * �򴮿ڷ�������
     *
     * @param serialPort ���ڶ���
     * @param data       ����������
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
     * �Ӵ��ڶ�ȡ����
     *
     * @param serialPort ��ǰ�ѽ������ӵ�SerialPort����
     * @return ��ȡ��������
     */
    private static byte[] readFromPort(SerialPort serialPort) {

        InputStream in = null;
        byte[] bytes = null;
        try {
            in = serialPort.getInputStream();
            //��ȡbuffer������ݳ���
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
     * ��Ӽ�����
     *
     * @param serialPort, ���ڶ���
     * @param listener    ���ڼ�����
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

