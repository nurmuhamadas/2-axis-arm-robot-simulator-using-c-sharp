using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Simulator_robot_planar_2_DoF
{
    public partial class Form1 : Form
    {
        // Properties of forward Kinematics
        private double L1 = 2.5, L2 = 2.5,
                        Teta1 = 0.0, Teta2 = 0.0,
                        x1 = 2.5, x2 = 5.0, 
                        y1 = 0.0, y2 = 0.0,
                        Phi = Math.PI;
        // Properties of invers kinematics
        private double L1_inv = 2.5, L2_inv = 2.5,
                        Teta1_inv1 = 90.0, Teta2_inv1 = 0.0,
                        Teta1_inv2 = 90.0, Teta2_inv2 = 0.0,
                        x1_inv1 = 0.0, x1_inv2 = 0.0, x2_inv = 0.0,
                        y1_inv1 = 2.5, y1_inv2 = 2.5, y2_inv = 5.0;

        public Form1()
        {
            InitializeComponent();
        }

        // ------------>>>>>>>>> FORWARD KINEMATIKA <<<<<<<<<<------------

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            // Avoid the input exceeding the limit -->> The limit is amount of the graph axis
            if ((double)numericUpDown1.Value + L2 > 5)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown1.Value = (decimal)L1; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                L1 = (double)numericUpDown1.Value;  // Replace value of L1 with input
                hitungEoe();    // Run the findEoe() method to drawing on the graph
            }
        }

        private void numericUpDown2_ValueChanged(object sender, EventArgs e)
        {
            // Avoid the input exceeding the limit -->> The limit is the graph axis
            if ((double)numericUpDown2.Value + L1 > 5)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown2.Value = (decimal)L2; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                L2 = (double)numericUpDown2.Value;  // Replace value of L2 with input
                hitungEoe();    // Run the findEoe() method to drawing on the graph
            }
        }

        private void numericUpDown3_ValueChanged(object sender, EventArgs e)
        {
            Teta1 = (double) numericUpDown3.Value; // Replace value of Teta1 with input
            hitungEoe();    // Run the findEoe() method to drawing on the graph
        }

        private void numericUpDown4_ValueChanged(object sender, EventArgs e)
        {
            Teta2 = (double)numericUpDown4.Value;   // Replace value of Teta1 with input
            hitungEoe();    // Run the findEoe() method to drawing on the graph
        }

        private void hitungEoe()
        {
            // Position of joint 0 (base/origin) is set at (0,0) through design properties
            // Find position of joint 1
            x1 = L1 * Math.Cos(Teta1 * Phi / 180);  // Find position of x1
            y1 = L1 * Math.Sin(Teta1 * Phi / 180);  // Find position of y1

            // Find position of joint 2 or End of Effector (EoE)
            x2 = x1 + L2 * Math.Cos((Teta1 + Teta2) * Phi / 180);   // Find position of x2
            y2 = y1 + L2 * Math.Sin((Teta1 + Teta2) * Phi / 180);   // Find position of y2

            // Replace value output of EoE in the text box
            textBox1.Text = Math.Round((decimal)x2, 4).ToString();
            textBox2.Text = Math.Round((decimal)y2, 4).ToString();

            // Drawing data on the graph
            double[] data1 = { x1, y1, x2, y2 };
            drawArm(0, 1, data1);
        }



        //--------------->>>>>>>>> INVERS KINEMATICS <<<<<<<<<<<--------------

        // Length of the arm
        private void numericUpDown8_ValueChanged(object sender, EventArgs e)
        {
            // Avoid the input exceeding the limit -->> The limit is the graph axis
            if ((double)numericUpDown8.Value + L2_inv > 5)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown8.Value = (decimal)L1_inv; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                L1_inv = (double)numericUpDown8.Value;  // Replace value of L1 with input
                numericUpDown5.Maximum = (decimal)(L1_inv + L2_inv);    // Set maximum input of EoE -> Maximum input is amount of L1 + L2
                numericUpDown6.Maximum = (decimal)(L1_inv + L2_inv);    // Set maximum input of EoE -> Maximum input is amount of L1 + L2
                findTeta();   // Run findTeta() method to find value of Teta and draw the amr on the graph
            }
        }

        private void numericUpDown7_ValueChanged(object sender, EventArgs e)
        {
            // Avoid the input exceeding the limit -->> The limit is the graph axis
            if ((double)numericUpDown7.Value + L1_inv > 5)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown8.Value = (decimal)L2_inv; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                L2_inv = (double)numericUpDown8.Value;  // Replace value of L2 with input
                numericUpDown5.Maximum = (decimal)(L1_inv + L2_inv);    // Set maximum input of EoE -> Maximum input is amount of L1 + L2
                numericUpDown6.Maximum = (decimal)(L1_inv + L2_inv);    // Set maximum input of EoE -> Maximum input is amount of L1 + L2
                findTeta();   // Run findTeta() method to find value of Teta and draw the arm on the graph
            }
        }

        private void numericUpDown6_ValueChanged(object sender, EventArgs e)
        {
            // Avoid the input exceeding the limit -->> The limit is distance of EoE with origin
            if (Math.Sqrt(Math.Pow((double)numericUpDown6.Value, 2) + Math.Pow(y2_inv, 2)) > L1_inv + L2_inv)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown6.Value = (decimal)x2_inv; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                x2_inv = (double)numericUpDown6.Value;  // Replace value of X2 with input
                findTeta();   // Run findTeta() method to find value of Teta and draw the arm on the graph
            }
        }

        private void numericUpDown5_ValueChanged(object sender, EventArgs e)
        {
            // Avoid input exceeding the limit -->> The limit is distance of EoE with origin
            if (Math.Sqrt(Math.Pow((double)numericUpDown5.Value, 2) + Math.Pow(x2_inv, 2)) > L1_inv + L2_inv)
            {
                // If exceeding the limit
                MessageBox.Show("You're overreaching! Repent immedeately", "PATIENT PLEASE!");  // Show the text box if the input exceeding the limit
                numericUpDown5.Value = (decimal)y2_inv; // Cancel the change
            }
            else
            {
                // If not exceeding the limit
                y2_inv = (double)numericUpDown5.Value;  // Replace value of X2 with input
                findTeta();   // Run findTeta() method to find value of Teta and draw the arm on the graph
            }
        }


        private void findTeta()
        {
            double xy = x2_inv * x2_inv + y2_inv * y2_inv;  // Find distance of EoE from origin
            if (x2_inv == 0) x2_inv = 0.00000000000001;     // Avoid error when y2/x2

            // >>> The first probability of Teta (P1) <<<
            Teta2_inv1 = Math.Acos(round((xy - L1_inv * L1_inv - L2_inv * L2_inv) / (2 * L1_inv * L2_inv))) * 180 / Phi;    // Find Teta 2 using cosinus law
            Teta1_inv1 = Math.Atan(y2_inv / x2_inv) * 180 / Phi - Math.Atan(L2_inv * Math.Sin(Teta2_inv1 * Phi / 180) / (L1_inv + L2_inv * Math.Cos(Teta2_inv1 * Phi / 180))) * 180 / Phi;  // Find Teta 1
            if (x2_inv < 0)
                Teta1_inv1 = 180 + Teta1_inv1;

            // Find position of joint 1
            x1_inv1 = L1_inv * Math.Cos(Teta1_inv1 * Phi / 180);  // Find x1
            y1_inv1 = L1_inv * Math.Sin(Teta1_inv1 * Phi / 180);  // Find y1

            // Show value on the text box
            textBox4.Text = Math.Round(Teta1_inv1, 3).ToString();
            textBox3.Text = Math.Round(Teta2_inv1, 3).ToString();

            // Draw data on the graph
            double[] data2 = { x1_inv1, y1_inv1, x2_inv, y2_inv };
            drawArm(2, 3, data2);

            // ------------------------------------------------------------------------------

            // >>> The second probability of Teta  (P2) <<<
            Teta2_inv2 = -(Math.Acos(round((xy - L1_inv * L1_inv - L2_inv * L2_inv) / (2 * L1_inv * L2_inv))) * 180 / Phi); // Find Teta 2 using cosinus law
            Teta1_inv2 = Math.Atan(y2_inv / x2_inv) * 180 / Phi - Math.Atan(L2_inv * Math.Sin(Teta2_inv2 * Phi / 180) / (L1_inv + L2_inv * Math.Cos(Teta2_inv2 * Phi / 180))) * 180 / Phi;  // Find Teta 1
            if (x2_inv < 0)
                Teta1_inv2 = 180 + Teta1_inv2;

            // Find position of joint 1
            x1_inv2 = L1_inv * Math.Cos(Teta1_inv2 * Phi / 180);  // Find x1
            y1_inv2 = L1_inv * Math.Sin(Teta1_inv2 * Phi / 180);  // Find y1

            // Show value on the text box
            textBox8.Text = Math.Round(Teta1_inv2, 3).ToString();
            textBox9.Text = Math.Round(Teta2_inv2, 3).ToString();

            // Draw data on the graph
            double[] data3 = { x1_inv2, y1_inv2, x2_inv, y2_inv };
            drawArm(4, 5, data3);
        }


        // ------------>>>>>>>>>> Some Functions <<<<<<<<<<-------------

        // Grid options
        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox1.Checked == true)
                chart1.ChartAreas[0].AxisX.MinorGrid.Enabled = true;
            else
                chart1.ChartAreas[0].AxisX.MinorGrid.Enabled = false;
        }

        private void drawArm(int line, int dot, double[] data) 
            // Params: line = select the arm, dot = select the joint, data = Posisiton of joint 1 and EoE
            //See more at design properties
        {
            // Draw data on the chart
            // Arm 1
            chart1.Series[line].Points[1].XValue = data[0];
            chart1.Series[line].Points[1].YValues[0] = data[1];
            chart1.Series[dot].Points[1].XValue = data[0];
            chart1.Series[dot].Points[1].YValues[0] = data[1];
            // Arm 2
            chart1.Series[line].Points[2].XValue = data[2];
            chart1.Series[line].Points[2].YValues[0] = data[3];
            chart1.Series[dot].Points[2].XValue = data[2];
            chart1.Series[dot].Points[2].YValues[0] = data[3];
        }

        // Avoid input of Asin, Acos > 1 or < -1
        private double round(double nilai)
        {
            if (nilai > 1) nilai = 1;
            if (nilai < -1) nilai = -1;
            return nilai;
        }
    }
}

/*
---------------->>>>>>>>>> DON'T REMOVE IT <<<<<<<<<<<<-----------------
Design and Develop by: Nur Muhamad Ash Shidiqi   
Mail to : nurmuhamad.a.13@gmail.com
Find more project at : https://github.com/nurmuhamadas
Visit : https://griyadeveloper.com

---------------->>>>>>>>>> THANK YOU <<<<<<<<<<<<-----------------
*/
