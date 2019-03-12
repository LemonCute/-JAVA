public class PositionCalculation {

    private double pi = Math.PI;
    private byte[] data1 = {0x7E, 0x11, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0, 0, (byte) 0xFF, 0x7F};
    private double x0 = 0.0000;            //��ǰλ��
    private double y0 = 0.0000;
    private double z0 = 0.000;
    private double a0 = 90.0000;
    private double b0 = 90.000;
    private double alpha;
    private double beta;
    private double gama;
    //����45���۳�210
    private int L0 = 45, L = 210, LP = 50;
    private double first_data[] = {0, 0, 0, 0, 0, 90, 90, 180};  //x0,y0,z0,a0,b0,alpha,beta,gama;
    private double second_data[] = {0, 0, 0, 90, 90, 90, 90, 180}; //x0,y0,z0,a0,b0,alpha,beta,gama;
    private double Zero_position[]={0,340,0};
    private double x_aim, y_aim, z_aim;

    public byte[] getData1() {
        return data1;
    }

    public double getX0() {
        return x0;
    }

    public double getY0() {
        return y0;
    }

    public double getZ0() {
        return z0;
    }

    public double getA0() {
        return a0;
    }

    public double getB0() {
        return b0;
    }

    public double[] getFirst_data() {
        return first_data;
    }

    public void setFirst_data(double[] first_data) {
        this.first_data = first_data;
    }

    public double[] getSecond_data() {
        return second_data;
    }

    public void setSecond_data(double[] second_data) {
        this.second_data = second_data;
    }

    public double getX_aim() {
        return x_aim;
    }

    public double getY_aim() {
        return y_aim;
    }

    public double getZ_aim() {
        return z_aim;
    }

    public double getAlpha() {
        return alpha;
    }

    public double getGama() {
        return gama;
    }

    public double getBeta() {
        return beta;
    }

    public double[] getZero_position() {
        return Zero_position;
    }

    public void setZero_position(double[] zero_position) {
        Zero_position = zero_position;
    }

    /**
     * @param d ������������
     */
    public void go_ahead(double d)     //���������˶�
    {
        double a1, b1;
        double cx, cy, cz;
        a1 = (a0 / 180.0) * pi;   //�Ƕ���ת��������
        b1 = (b0 / 180.0) * pi;

        cx = d * Math.cos(b1) * Math.cos(a1);    //������������
        cy = -d * Math.cos(b1) * Math.sin(a1);
        cz = -d * Math.sin(b1);


        x0 = x0 + cx;     //Ŀ��ֵ�޸ĵ�ǰֵ
        y0 = y0 + cy;
        z0 = z0 + cz;
        if ((cx - 0) > 0) data1[3] = 1;     //�жϸ�������λ ȷ���������
        else data1[3] = 0;
        if ((cy - 0) > 0) data1[7] = 1;
        else data1[7] = 0;
        if ((cz - 0) > 0) data1[11] = 1;
        else data1[11] = 0;
        data1[4] = (byte) (((int) (Math.abs(cx) * 100)) / 256);      //ȷ������������˶����� ����ÿ���������16λ���� ��С����10um
        data1[5] = (byte) (((int) (Math.abs(cx) * 100)) % 256);
        data1[8] = (byte) (((int) (Math.abs(cy) * 100)) / 256);
        data1[9] = (byte) (((int) (Math.abs(cy) * 100)) % 256);
        data1[12] = (byte) (((int) (Math.abs(cz) * 100)) / 256);
        data1[13] = (byte) (((int) (Math.abs(cz) * 100)) % 256);
        data1[16] = 0;
        data1[17] = 0;
        data1[20] = 0;
        data1[21] = 0;
        data1[22] = 0;
    }

    /**
     * @param num       �����
     * @param fangxiang ����˶�����
     * @param step      �˶�����
     */
    public void unit(int num, int fangxiang, double step)     //���������˶�����
    {
        double ca, cb, cx, cy, cz;
        double aa, bb;
        double l, a1, b1, c1;      //l���ڼ��㵼��ĩ���붨λ�����
        a1 = x_aim - (x0 + L * Math.sin(a0));
        b1 = y_aim - (y0 + L * Math.cos(a0));
        c1 = z_aim - z0;
        l = Math.sqrt(a1 * a1 + b1 * b1 + c1 * c1);
        if (num == 3) {
//����������ƫת�˶�
            if (fangxiang > 0) ca = step;   //����ƫת�ǲ�ֵ
            else ca = -step;
            aa = a0 + ca;
            a0 = (a0 / 180.0) * pi;     //�Ƕ���ת��������
            aa = (aa / 180.0) * pi;
            b0 = (b0 / 180.0) * pi;
            cx = l * Math.cos(b0) * Math.cos(aa) - L * Math.sin(aa) - (l * Math.cos(b0) * Math.cos(a0) - L * Math.sin(a0));    //������������
            cy = 0 - l * Math.cos(b0) * Math.sin(aa) - L * Math.cos(aa) - (0 - l * Math.cos(b0) * Math.sin(a0) - L * Math.cos(a0));
            a0 = (a0 / pi) * 180;       //������ת���Ƕ���
            aa = (aa / pi) * 180;
            b0 = (b0 / pi) * 180;
            if (aa > 180) a0 = aa - 360;     //Ŀ��ֵ�޸ĵ�ǰֵ
            else if (aa < -180) a0 = aa + 360;
            else a0 = aa;

            if ((cx - 0) > 0) data1[3] = 1;    //�жϸ�������λ ȷ���������
            else data1[3] = 0;
            if ((cy - 0) > 0) data1[7] = 1;
            else data1[7] = 0;
            if (fangxiang > 0) data1[15] = 0;
            else data1[15] = 1;
            if (fangxiang > 0) data1[19] = 0;
            else data1[19] = 1;
            data1[4] = (byte) (((int) (Math.abs(cx) * 100)) / 256);         //ȷ������������˶����� ����ÿ���������16λ���� ��С����10um
            data1[5] = (byte) (((int) (Math.abs(cx) * 100)) % 256);
            data1[8] = (byte) (((int) (Math.abs(cy) * 100)) / 256);
            data1[9] = (byte) (((int) (Math.abs(cy) * 100)) % 256);
            data1[16] = (byte) (((int) (Math.abs(ca) * 100)) / 256);     //ȷ���Ƕȱ仯��16λ���䣬��С����0.01��
            data1[17] = (byte) (((int) (Math.abs(ca) * 100)) % 256);
            data1[12] = 0;
            data1[13] = 0;
            data1[20] = 0;
            data1[21] = 0;
            data1[22] = 0;
        }
        if (num == 4) {
//���������㸩���˶�
            if (fangxiang > 0) cb = step;   //����ƫת�ǲ�ֵ
            else cb = -step;
            bb = b0 + cb;
            b0 = (b0 / 180.0) * pi;   //�Ƕ���ת��������
            bb = (bb / 180.0) * pi;
            a0 = (a0 / 180.0) * pi;
            cx = l * (Math.cos(bb) - Math.cos(b0)) * Math.cos(a0);   //������������
            cy = 0 - l * (Math.cos(bb) -
                    Math.cos(b0)) * Math.sin(a0);
            cz = l * (Math.sin(bb) - Math.sin(b0));
            b0 = (b0 / pi) * 180;      //������ת���Ƕ���
            bb = (bb / pi) * 180;
            a0 = (a0 / pi) * 180;
            if (bb > 180) b0 = bb - 360;     //Ŀ��ֵ�޸ĵ�ǰֵ
            else if (bb < -180) b0 = bb + 360;
            else b0 = bb;

            if ((cx - 0) > 0) data1[3] = 1;    //�жϸ�������λ ȷ���������
            else data1[3] = 0;
            if ((cy - 0) > 0) data1[7] = 1;
            else data1[7] = 0;
            if ((cz - 0) > 0) data1[11] = 1;
            else data1[11] = 0;
            if (fangxiang > 0) data1[19] = 0;
            else data1[19] =1;
            if (fangxiang > 0) data1[15] = 0;
            else data1[15] =1;
            data1[4] = (byte) (((int) (Math.abs(cx) * 100)) / 256);            //ȷ������������˶����� ����ÿ���������16λ���� ��С����10um
            data1[5] = (byte) (((int) (Math.abs(cx) * 100)) % 256);
            data1[8] = (byte) (((int) (Math.abs(cy) * 100)) / 256);
            data1[9] = (byte) (((int) (Math.abs(cy) * 100)) % 256);
            data1[12] = (byte) (((int) (Math.abs(cz) * 100)) / 256);
            data1[13] = (byte) (((int) (Math.abs(cz) * 100)) % 256);
            data1[20] = (byte) (((int) (Math.abs(cb) * 100)) / 256);            //ȷ���Ƕȱ仯��16λ���䣬��С����0.01��
            data1[21] = (byte) (((int) (Math.abs(cb) * 100)) % 256);
            data1[16] = 0;
            data1[17] = 0;
            data1[22] = 0;
        }
    }

    /**
     *
     * @param roll
     * @param heading
     * @param pitch
     */
    public void change(double roll, double heading, double pitch)    //���᷽�����䷨������ת��
    {
        double hea, pit, ro;      //�м����
        heading = -heading;
        hea = (heading / 180.0) * pi;
        pit = (pitch / 180.0) * pi;
        ro = (roll / 180.0) * pi;
        alpha = Math.acos(Math.cos(ro) * Math.sin(pit) * Math.cos(hea));
        beta = Math.acos(Math.cos(ro) * Math.sin(pit) * Math.cos(hea + pi / 2));//����ط�Ҫ��
        gama = Math.acos(Math.cos(ro) * Math.cos(pit));
        alpha = (alpha / pi) * 180;
        beta = (beta / pi) * 180;
        gama = 180 - (gama / pi) * 180;
    }

    /**
     * @param first  ��һ�α�������
     * @param second �ڶ��α�������
     */
    public void find(double[] first, double[] second)     //Ѱ�Ҷ�λ��
    {
        double a, b, c;   //��һ��Ͷ����ȷ��ƽ��ķ�������a,b,c��
        double d;   //��λ�������͵�ľ���
        double def, pit; //�Ƕ�ת���м����
        double i, j, k;
        def = (first[3] / 180.0) * pi;
        pit = (first[4] / 180.0) * pi;
        double m1 = first[0] + L * Math.sin(def), n1 = first[1] + L * Math.cos(def), k1 = first[2];
        double arfa1 = Math.acos(Math.cos(pit) * Math.cos(def)), beta1 = pi - Math.acos(Math.cos(pit) * Math.sin(def));
        double gama1 = 0;

        if ((pit > 0 && pit <= pi / 2.0) || (pit > 3 * pi / 2.0 && pit <= 2 * pi)) {
            gama1 = pi / 2.0 + pit;  //����ǰ�ĸ�����ת���ɵ�ǰ����z��ļн�
        }
        if (pit > pi / 2.0 && pit < 3 * pi / 2.0) {
            gama1 = pi / 2.0 + pi - pit;
        }

        double arfa61 = (first[5] / 180.0) * pi, beta61 = (first[6] / 180.0) * pi, gama61 = (first[7] / 180.0) * pi;
        def = (second[3] / 180.0) * pi;
        pit = (second[4] / 180.0) * pi;
        double m2 = second[0] + L * Math.sin(def), n2 = second[1] + L * Math.cos(def), k2 = second[2];
        double arfa2 = Math.acos(Math.cos(pit) * Math.cos(def)), beta2 = pi - Math.acos(Math.cos(pit) * Math.sin(def));
        double gama2 = 0;

        if ((pit > 0 && pit <= pi / 2.0) || (pit > 3 * pi / 2.0 && pit <= 2 * pi)) {
            gama2 = pi / 2.0 + pit;
        }
        if (pit > pi / 2.0 && pit < 3 * pi / 2.0) {
            gama2 = pi / 2.0 + pi - pit;
        }

        double arfa62 = (second[5] / 180.0) * pi, beta62 = (second[6] / 180.0) * pi, gama62 = (second[7] / 180.0) * pi;

        i = Math.cos(beta1) * Math.cos(gama61) - Math.cos(gama1) * Math.cos(beta61);
        j = Math.cos(gama1) * Math.cos(arfa61) - Math.cos(arfa1) * Math.cos(gama61);
        k = Math.cos(arfa1) * Math.cos(beta61) - Math.cos(beta1) * Math.cos(arfa61);

        a = Math.cos(beta1) * Math.cos(gama61) - Math.cos(gama1) * Math.cos(beta61);   //��÷�����
        b = -(Math.cos(arfa1) * Math.cos(gama61) - Math.cos(gama1) * Math.cos(arfa61));
        c = Math.cos(arfa1) * Math.cos(beta61) - Math.cos(beta1) * Math.cos(arfa61);

        System.out.println(a +" "+b+""+c);
        d = -(a * (m2 - m1 + LP * Math.cos(arfa2)) + b * (n2 - n1 + LP * Math.cos(beta2)) + c * (k2 - k1 + LP * Math.cos(gama2))) / (a * Math.cos(arfa62) + b * Math.cos(beta62) + c * Math.cos(gama62));
        x_aim = m2 + LP * Math.cos(arfa2) + d * Math.cos(arfa62);
        y_aim = n2 + LP * Math.cos(beta2) + d * Math.cos(beta62);
        z_aim = k2 + LP * Math.cos(gama2) + d * Math.cos(gama62);

    }
//
//    /**
//     * @param a Ŀ����x����
//     * @param b Ŀ����y����
//     * @param c Ŀ����z����
//     */
//    public void point(double a, double b, double c)       //ʹ����ָ��λ��
//    {
//        double x1 = 0, y1 = 0, a1, b1, c1, s1, s;        //x1��y1Ϊָ�����������ꣻa1��b1��c1��s1����������ߵ�λ������ʹ�ã�s���ж϶�λ�����z�������ʹ��
//        double def1 = 0, pit1, ang, ang_p;        //def1,pit1����ȷ������ָ��������ƫת�Ǻ͸����ǣ�ang�Ƕ�λ���������Բ�����������ε��ڲ��Ƕȣ�ang_p�Ƕ�λ����z�����ߺ���y��ļн�
//        double ca, cb, cx, cy;    //�������ֵ
//
//        s = Math.sqrt(a * a + b * b);
//        ang_p = Math.acos(b / s);
//        if (s > L) {
//            ang = Math.acos(L / s);
//            if(b>(0-L)&&b<L&&a<0)     //�����ж����ߵ��е��Ƿ���x�������ᣬ�Ӷ��ж�def1������
//                def1=-(ang_p-ang);
//            else
//            {
//                if(a<0&&b>0)
//                    def1=ang-ang_p;
//                if(a>0&&b>0)
//                    def1=ang+ang_p;
//                if(a>0&&b<0)
//                    def1=ang_p-ang;
//                if(a<0&&b<0)
//                    def1=2*pi-(ang+ang_p);
//            }
//
//
//            a1 = a - L * Math.sin(def1);
//            b1 = b - L * Math.cos(def1);
//            c1 = c - z0;
//            s1 = Math.sqrt(a1 * a1 + b1 * b1 + c1 * c1);
//            c1 = c1 / s1;   //������ߵ�λ����
//            if((ang_p>def1&&def1>0&&a>0)||(def1>pi/2&&a<0))   //ȷ��pit1�ĳ���
//            {
//                if(Math.acos(c1)<pi/2)
//                    pit1=2*pi-(pi/2-Math.acos(c1));
//                else
//                    pit1=pi/2-(pi-Math.acos(c1));
//            }
//            else
//            {
//                if(Math.acos(c1)<pi/2)
//                    pit1=pi+(pi/2-Math.acos(c1));
//                else
//                    pit1=pi-(Math.acos(c1)-pi/2);
//            }
//        } else        //��λ���ڱ�Բ�ڲ�
//        {
//            if (a < 0) {
//                ang_p = -ang_p;
//            }
//            x1 = 0 - (L - s) * Math.sin(ang_p) - L0 * Math.cos(ang_p);     //�����ƶ������꣬�����λ���ڱ�Բ�ڲ�����ô�����ƶ���ʹ֮��Ȧ�����
//            y1 = 0 - (L - s) * Math.cos(ang_p) - L0 * Math.sin(ang_p);
//            def1 = ang_p;
//            a1 = a - (x1 + L * Math.sin(def1));
//            b1 = b - (y1 + L * Math.cos(def1));
//            c1 = c - z0;
//            s1 = Math.sqrt(a1 * a1 + b1 * b1 + c1 * c1);
//            c1 = c1 / s1;   //������ߵ�λ����
//            if(Math.acos(c1)<pi/2)
//                pit1=2*pi-(pi/2-Math.acos(c1));
//            else
//                pit1=pi/2-(pi-Math.acos(c1));
//        }
//        cx = x1 - x0;     //��ֵ
//        cy = y1 - y0;
//        def1 = (def1 / pi) * 180;
//        pit1 = (pit1 / pi) * 180;
//        ca = def1 - a0;
//        cb = pit1 - b0;
//        x0 = x1;    //�޸ĵ�ǰֵ
//        y0 = y1;
//        a0 = def1;
//        b0 = pit1;
//        if ((cx - 0) > 0) data1[3] = 1;    //�жϸ�������λ ȷ���������
//        else data1[3] = 0;
//        if ((cy - 0) > 0) data1[7] = 1;
//        else data1[7] = 0;
//        if ((ca - 0) > 0) data1[19] = 1;
//        else data1[19] = 0;
//        if ((cb - 0) > 0) data1[15] = 0;
//        else data1[15] = 1;
//
//        if (Math.abs(ca) > 180)     //�ж�ת���Ƕ� ����Ƕȴ���180����ô������ת
//        {
//            ca = 360 - Math.abs(ca);
//            if(data1[19]==0){
//                data1[19]=1;
//            }else{
//                data1[19]=0;
//            }
//        }
//        if (Math.abs(cb) > 180) {
//            cb = 360 - Math.abs(cb);
//            if(data1[15]==0){
//                data1[15]=1;
//            }else{
//                data1[15]=0;
//            }
//        }
//        data1[4] = (byte) (((int) (Math.abs(cx) * 100)) / 256);
//        data1[5] = (byte) (((int) (Math.abs(cx) * 100)) % 256);
//        data1[8] = (byte) (((int) (Math.abs(cy) * 100)) / 256);
//        data1[9] = (byte) (((int) (Math.abs(cy) * 100)) % 256);
//        data1[16] = (byte) (((int) (Math.abs(ca) * 100)) / 256);
//        data1[17] = (byte) (((int) (Math.abs(ca) * 100)) % 256);
//        data1[12] = 0;
//        data1[13] = 0;
//        data1[20] = (byte) (((int) (Math.abs(cb) * 100)) / 256);
//        data1[21] = (byte) (((int) (Math.abs(cb) * 100)) % 256);
//        data1[22] = 0;
//    }

    /**
     * @param a Ŀ����x����
     * @param b Ŀ����y����
     * @param c Ŀ����z����
     */
    void point(double a, double b, double c)       //ʹ����ָ��λ��
    {
        double xx,yy,a1,b1,c1,s1,s;        //xx��yyΪָ�����������ꣻa1��b1��c1��s1����������ߵ�λ������ʹ�ã�s���ж϶�λ�����z�������ʹ��
        double def1,pit1,arfa1,ang_p;        //def1,pit1����ȷ������ָ��������ƫת�Ǻ͸����ǣ�arfal�Ƕ�λ����۶������߽Ƕȣ�ang_p�Ƕ�λ����z�����ߺ���y��ļн�
        double ca,cb,cx,cy;    //�������ֵ
        double x1,y1,z1,ha0; //x1,y1,z1�Ǳ۶�������

        ha0 = a0*pi/180;
        x1 = x0 + Math.sin(ha0)*L;
        y1 = y0 + Math.cos(ha0)*L;
        z1 = z0;

        arfa1 = Math.atan((b-y1)/(a-x1));
        s=Math.sqrt((a-x0)*(a-x0)+(b-y0)*(b-y0));
        ang_p=Math.acos((b-y0)/s);


        a1 = a-x1;
        b1 = b-y1;
        c1 = c-z1;
        s1 = Math.sqrt(a1 * a1 + b1 * b1 + c1 * c1);
        c1 = c1 / s1;

        if(a>x0)     //�����ж����ߵ��е��Ƿ���x�������ᣬ�Ӷ��ж�def1������
            if(a0>0)
                if(ang_p-ha0>0)
                {
                    def1= -arfa1;
                    if(Math.acos(c1)<pi/2)
                        pit1=2*pi-(pi/2-Math.acos(c1));
                    else
                        pit1=pi/2-(pi-Math.acos(c1));
                }
                else
                {
                    def1= pi-arfa1;
                    if(Math.acos(c1)<pi/2)
                        pit1=pi+(pi/2-Math.acos(c1));
                    else
                        pit1=pi-(Math.acos(c1)-pi/2);
                }
            else
            if(ang_p-ha0>pi)
            {
                def1= -pi-arfa1;
                if(Math.acos(c1)<pi/2)
                    pit1=pi+(pi/2-Math.acos(c1));
                else
                    pit1=pi-(Math.acos(c1)-pi/2);
            }
            else
            {
                def1= -arfa1;
                if(Math.acos(c1)<pi/2)
                    pit1=2*pi-(pi/2-Math.acos(c1));
                else
                    pit1=pi/2-(pi-Math.acos(c1));
            }
        else
        if(a0>0)
            if(ang_p+ha0>pi)
            {
                def1= -pi-arfa1;
                if(Math.acos(c1)<pi/2)
                    pit1=2*pi-(pi/2-Math.acos(c1));
                else
                    pit1=pi/2-(pi-Math.acos(c1));
            }
            else
            {
                def1= -arfa1;
                if(Math.acos(c1)<pi/2)
                    pit1=pi+(pi/2-Math.acos(c1));
                else
                    pit1=pi-(Math.acos(c1)-pi/2);
            }
        else
        if(ang_p+ha0>0)
        {
            def1= -arfa1;
            if(Math.acos(c1)<pi/2)
                pit1=pi+(pi/2-Math.acos(c1));
            else
                pit1=pi-(Math.acos(c1)-pi/2);
        }
        else
        {
            def1= -pi-arfa1;
            if(Math.acos(c1)<pi/2)
                pit1=2*pi-(pi/2-Math.acos(c1));
            else
                pit1=pi/2-(pi-Math.acos(c1));
        }

        xx = x1 - Math.sin(def1)*L;
        yy = y1 - Math.cos(def1)*L;
        cx = xx - x0;     //��ֵ
        cy = yy - y0;
        def1 = (def1 / pi) * 180;
        pit1 = (pit1 / pi) * 180;
        ca = def1 - a0;
        cb = pit1 - b0;
        x0= xx;          //�޸ĵ�ǰֵ
        y0 = yy;
        a0 = def1;
        b0 = pit1;
        if ((cx - 0) > 0) data1[3] = 1;    //�жϸ�������λ ȷ���������
        else data1[3] = 0;
        if ((cy - 0) > 0) data1[7] = 1;
        else data1[7] = 0;
        if ((ca - 0) > 0) data1[19] = 1;
        else data1[19] = 0;
        if ((cb - 0) > 0) data1[15] = 0;
        else data1[15] = 1;

        if (Math.abs(ca) > 180)     //�ж�ת���Ƕ� ����Ƕȴ���180����ô������ת
        {
            ca = 360 - Math.abs(ca);
            if(data1[19]==0){
                data1[19]=1;
            }else{
                data1[19]=0;
            }
        }
        if (Math.abs(cb) > 180) {
            cb = 360 - Math.abs(cb);
            if(data1[15]==0){
                data1[15]=1;
            }else{
                data1[15]=0;
            }
        }
        data1[4] =(byte)((Math.abs(cx) * 100) / 256);
        data1[5] =(byte)((Math.abs(cx) * 100) % 256);
        data1[8] =(byte)((Math.abs(cy) * 100) / 256);
        data1[9] =(byte)((Math.abs(cy) * 100) % 256);
        data1[16] =(byte)((Math.abs(ca) * 100) / 256);
        data1[17] =(byte)((Math.abs(ca) * 100) % 256);
        data1[12] = 0;
        data1[13] = 0;
        data1[20] =(byte)((Math.abs(cb) * 100)/ 256);
        data1[21] =(byte) ((Math.abs(cb) * 100) % 256);
        data1[22] = 0;
    }
    /**
     * @param MotorNumber �����
     * @param Direction   ����
     * @param Distance    ����
     */
    public void move(int MotorNumber, int Direction, double Distance) {

        if (MotorNumber == 0) {
            if (Direction == 0) {
                x0-=Distance;
            } else if (Direction == 1) {
                x0+=Distance;
            }
        } else if (MotorNumber == 1) {
            if (Direction == 0) {
                y0-=Distance;
            } else if (Direction == 1) {
                y0+=Distance;
            }
        } else if (MotorNumber == 2) {
            if (Direction == 0) {
                z0-=Distance;
            } else if (Direction == 1) {
                z0+=Distance;
            }
        } else if (MotorNumber == 3) {
            if (Direction == 0) {
                a0-=Distance;
            } else if (Direction == 1) {
                a0+=Distance;
            }
        } else if (MotorNumber == 4) {
            if (Direction == 0) {
                b0+=Distance;
            } else if (Direction == 1) {
                b0-=Distance;
            }
        }

    }
}


