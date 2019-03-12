public class OptionSystem {

    /*
     * -----------标志位声明--------------------------
     * @Step;电机步长
     * @SaveNum：数据保存次数---》用于模式一的定位点计算
     * @Positioning_success：判断机械臂是否已经指向定位点
     */
    static private double Step = 0.40;//电机的步进步长
    static private double Step_G0 =1;//下扎时的电机的步进步长
    static private int SaveNum = 1;
    static private boolean Positioning_success = false;

    public static void main(String[] args) {

        System.out.println("\nThe operation system is start!!!!");

        // 新建类的对象
        SerialPort_Manager SerialManager = new SerialPort_Manager();
        PositionCalculation position = new PositionCalculation();

        /*
         *开机自动打开串口进行监听如果串口未打开，等待串口打开
         */
        while (!SerialManager.isSerialPortIsOpen()) {
            //如果发现没有可用串口，输出提示，请求插入插入串口
           //如果有可用串口，默认打开第一个
            if (SerialManager.findPort().isEmpty()) {
                System.out.println("please insert the serial");
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            } else {
                /*
                 *如果串口连接，打开串口，并打印信息
                 * Serialmanager.setSerialPortIsOpen：将标志位置起，停止循环
                 */
                SerialManager.openPort(SerialManager.findPort().get(0), 115200);
                SerialManager.setSerialPortIsOpen(true);

                System.out.println("Serialport is open !!!!" + SerialManager.findPort().get(0));
                // Serialmanager.openPort("/dev/ttyAMA2", 115200);
                //System.out.println("Serialport is open !!!!");

            }
        }
        /*
         * 开机获取六轴零位角度
         * Serialmanager.isZeroPosition()：是否得到零位角度
         * true：
         * ---提示得到信息，置起标志位停止循环
         * ---将角度缓存区中的角度取出，并保存 position.setZero_position(Angel);
         * false：打印信息 ，等待零位获取，知道
         *
         */
        while (!SerialManager.isZeroPosition()) {
            /**
             * 没有得到零位角度，循环等待
             */
            System.out.println("The Zero Position has not been obtained ");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (SerialManager.isZeroPosition()) {
            /*
             * 用Angel数组来接受六轴的角度，方便判断和赋值
             */
            double Angel[] = {Double.valueOf(SerialManager.getAngel()[2]),
                    Double.valueOf(SerialManager.getAngel()[3]),
                    Double.valueOf(SerialManager.getAngel()[4])};
            SerialManager.setZeroPosition(true);
            position.setZero_position(Angel);
            String moving[]={"mode","1","0","0","0"};
            SerialManager.setMoving(moving);
            System.out.println("the Zero angel is Get");

        }

        /*
         * 主程序循环执行，判断各个状态为来选择是否执行相应的命令
         */
        while (true) {
            /*
             * @ Serialmanager.isDataAvalible() 串口有数据标志位
             * @ 如果串口有数据，就进行状态的判断，否则就进行短暂的休眠
             */
            if (SerialManager.isDataAvalible()) {
                //@将串口的数据标志清除
                SerialManager.setDataAvalible(false);
                //@从机械臂动作缓存中取出动作缓存
                String[] moving = SerialManager.getMoving();
                /*
                 *  @MoveMode 运动模式标志位
                 *  MoveMode==1--->寻找定位点运动模式
                 *  MoveMode==2--->手动调整运动模式
                 */
                int MoveMode = Integer.valueOf(moving[1]);
                /*
                 * 如果选择的是模式一：
                 * ---->如果选择的是保存数据：Serialmanager.isSavePosition()==true
                 *      将从六轴的角度缓冲区中取出相关角度并保存在对应的数组中（前提是保存的时候会将相关角度传进对应的数组中）
                 *      @SaveNum--保存的次数
                 *         --->SaveNum==1,将数据保存在position的First_data[];
                 *         --->SaveNum==2,将数据保存在position的Sencond_data[];
                 *         --->SaveNum==3,进行定位点的计算
                 *         --->SaveNum==4,手动调整完成，提示进入自动指向过程
                 *---->如果选择的是运动完成，进行数据的更新：
                 *     条件：Serialmanager.isSavePosition()==false&& Serialmanager.isMoveIsFinish()==true
                 *     将position中的对应的变量进行修改
                 */
                if (MoveMode == 1) {
                  //  System.out.println("The option mode is 1");

                    //保存模式
                    if (SerialManager.isSavePosition()) {
                        //System.out.println("this is save mode");
                        //@ 清除保存选项标志位
                        SerialManager.setSavePosition(false);
                        /*
                         *该判断的存在是为了防止出现数据丢包而接受到的角度值为零的情况
                         */
                        double Angel[] = {Double.valueOf(SerialManager.getAngel()[2]),
                                Double.valueOf(SerialManager.getAngel()[3]),
                                Double.valueOf(SerialManager.getAngel()[4])};
                        /*
                         * 如果获得的角度没有错误，就将所有的数据进行保存
                         */
                            position.change(Angel[0] - position.getZero_position()[0],
                                    Angel[1] - position.getZero_position()[1],
                                    Angel[2] - position.getZero_position()[2]);
                            System.out.println("the angel has been  transfromed");
                            /*
                             *SaveNum用来判断保存的进行的次数，以获得当前程序的执行情况
                             * */
                            if (SaveNum == 1) {
                                SaveNum = 2;
                                /*
                                 *从缓存中取出各个坐标值，进行一次状态保存
                                 */
                                double[] data = {position.getX0(), position.getY0(), position.getZ0(),
                                        position.getA0(), position.getB0(),
                                        position.getAlpha(), position.getBeta(), position.getGama()};
                                //将获得的第一个定位点的各个坐标的实际值，保存到First_data数组中
                                position.setFirst_data(data);
                                System.out.println("The Frist Data is Saved");
                                //如果程序是第二次运行，那么在此步骤将后面的自动指向完成标志清零，以达到程序可以循环使用
                                if(Positioning_success){
                                    Positioning_success=false;
                                }
                            } else if (SaveNum == 2) {
                                SaveNum = 3;
                                 //从缓存中取出各个坐标值，进行一次状态保存
                                double[] data = {position.getX0(), position.getY0(), position.getZ0(),
                                        position.getA0(), position.getB0(),
                                        position.getAlpha(), position.getBeta(), position.getGama()};
                                position.setSecond_data(data);
                                System.out.println("The data has been get,please press the ok to Calculate  ");
                            }else if(SaveNum == 3){
                                SaveNum=4;
                                position.find(position.getFirst_data(), position.getSecond_data());

                                System.out.println("The Aim distance is Calculated ");
                                System.out.println("----------------------------------------");
                                System.out.println("the aim data is ->");
                                System.out.println(position.getX_aim());
                                System.out.println(position.getY_aim());
                                System.out.println(position.getZ_aim());
                                System.out.println("----------------------------------------");
                                System.out.println("You can do the adjust");
                            } else if (SaveNum==4) {
                                System.out.println("Please Switch the mode to point ");
                            }
                    } else if (SerialManager.isMoveIsFinish() && !SerialManager.isSavePosition()) {
                            /*
                             * @MotorNumber：电机号
                             * @Direction：方向
                             * @Distance:电机运动的距离
                             */
                            int MotorNumber = Integer.valueOf(moving[2]);
                            int Direction = Integer.valueOf(moving[3]);
                            double Distance = Double.valueOf(moving[4]);
                            /*
                             *  position.move():改变对应的坐标参数
                             */
                            position.move(MotorNumber, Direction, Distance / 100);
                            SerialManager.setMoveIsFinish(false);
                            System.out.println("the distance has been saved");
                        }
                    }
                    /*
                     * 如果选择的是模式二：
                     * --->按确定后进行自动指向操作，并将Positioning_success置为true，代表定位操作已经完成
                     * @Serialmanager.isAutoMove()是否执行自动指向定位点操作
                     * @Positioning_success：是否完成定位指向
                     * @ position.find():计算定位点的函数
                     * @ position.point90：计算机械臂电机运动过程的函数
                     * @ Serialmanager.sendToPort(Serialmanager.getSerialPort(), position.getData1());
                     *  将计算出来的运动缓冲数组发送给下位机
                     */
                    else if (MoveMode == 2) {
                        System.out.println("The option mode is 2");
                        /*
                         * 程序必须指向定位点后才能够继续执行，如果没有指向定位点，
                         * 则需要按下确定键，来让机械臂指向定位点
                         */
                        if (!Positioning_success) {

                            /*
                             * @Serialmanager.isAutoMove()：判断是否按下确定键
                             *  如果按下确定键，则调用函数，计算机械路径，指挥下位机运动
                             * @position.find（）：寻找定位点
                             * @position.point（）：计算指向定位点路径
                             */
                            if (SerialManager.isAutoMove()) {
                                /*
                                 * 将标志位清除，是程序可重复进行
                                 */
                                SerialManager.setAutoMove(false);
                                position.point(position.getX_aim(), position.getY_aim(), position.getZ_aim());
                                /*
                                 *@ Serialmanager.getSerialPort():取得已经打开的串口号
                                 *@position.getData1()：取得下一个动作的缓存
                                 *@Serialmanager.sendToPort（）：发送到串口
                                 */
                                SerialManager.sendToPort(SerialManager.getSerialPort(), position.getData1());
                                Positioning_success = true;SaveNum=1;
                                /*
                                 *指向定位点成功后，将标志位置起，使程序可以进行其他操作
                                 */
                                System.out.println("This is AutoMove Mode");
                                System.out.println("----------------------------------------");
                                System.out.println("The First Saved Data is ->");
                                for(int i=0;i<position.getFirst_data().length;i++){
                                     System.out.print(position.getFirst_data()[i]+" ");
                                 }
                                System.out.println("\r\n----------------------------------------");
                                System.out.println("The Second Saved Data is ->");
                                for(int i=0;i<position.getSecond_data().length;i++){
                                     System.out.print(position.getSecond_data()[i]+" ");
                                }
                                System.out.println("\r\n----------------------------------------");
                                System.out.println("The send Data is ->");
                                for(int i=0;i<position.getData1().length;i++){
                                     System.out.print(position.getData1()[i]+" ");
                                }
                                System.out.println("\r\n----------------------------------------");
                            } else {
                                /*
                                 *如果机械臂还未指向定位点，不允许进行其他操作，并给予警告
                                 */
                                System.out.println("Please press the ok to move to the point");
                            }
                        } else {
                            /*
                             * 如果已经指向定位点，接下来进行的是手动调整下针方向的操作
                             * @MotorNumber：电机号
                             * @Direction：方向
                             * @Step:电机每次运动的固定步长
                             */
                            System.out.println("Calculation the next operation");
                            /*
                             * 获取电机号，和方向
                             */
                            int MotorNumber = Integer.valueOf(moving[2]);
                            int Direction = Integer.valueOf(moving[3]);
                            if (MotorNumber == 55) {
                                /*
                                 * position.go_ahead(Step);：计算下扎运动的动作
                                 */
                                if (Direction == 1) {
                                    position.go_ahead(Step_G0);
                                    SerialManager.sendToPort(SerialManager.getSerialPort(), position.getData1());
                                } else if (Direction == 0) {
                                    position.go_ahead(-Step_G0);
                                    SerialManager.sendToPort(SerialManager.getSerialPort(), position.getData1());
                                }
                            } else{
                                /*
                                 * position.unit（）：计算联合运动的动作
                                 */
                                position.unit(MotorNumber, Direction, 0.2);
                                /*
                                 * 发送到串口，指挥下位机进行操作
                                 */
                                SerialManager.sendToPort(SerialManager.getSerialPort(), position.getData1());

                            }
                        }
                    }
            } else {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}




