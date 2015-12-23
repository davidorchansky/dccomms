package paquete;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.TextField;

import javax.swing.*;
import javax.swing.text.DefaultCaret;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartFrame;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.labels.StandardCategoryItemLabelGenerator;
import org.jfree.chart.labels.StandardXYItemLabelGenerator;
import org.jfree.chart.labels.XYItemLabelGenerator;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.time.Millisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RectangleInsets;



public class Vista implements IrssiBaliza0ButtonObserver, IrssiBaliza1ButtonObserver, IrxObserver, IGraficaObserver{
	private JTextArea rx;
	private JFrame frame;
	private JPanel northSouthPanel;
	private JPanel botones;
	private JButton rssiBaliza0, rssiBaliza1, limpiarDatos, delUltMuestra;
	private JTextField distInput;
	private MyTextField yLower;
	private MyTextField yUpper;
	private MyTextField xLower;
	private MyTextField xUpper;
	private JPanel southPanel;
	private JPanel ejeXpanel;
	private JPanel ejeYpanel;
	private JPanel nmuestrasPanel;
	private JCheckBox ejeXfijo;
	private JCheckBox ejeYfijo;
	private JCheckBox capturaContinua;
	//private JPanel capturaContinuaPanel;
	private XYSeries rssiIda = new XYSeries("RSSI ida");
	private XYSeries rssiVuelta = new XYSeries("RSSI vuelta");
	private XYSeriesCollection dataset =new XYSeriesCollection();
	
	//private TimeSeries rssiIdaCont = new TimeSeries("RSSI ida",Millisecond.class);
	private TimeSeries rssiIdaCont = new TimeSeries("RSSI ida");
	private TimeSeries rssiVueltaCont =new TimeSeries("RSSI vuelta");
	private TimeSeriesCollection datasetCont = new TimeSeriesCollection();
	
	private JFreeChart chart;
	private ChartFrame chartFrame;
	private Modelo modelo;
	
	private TextField numMuestras=new TextField("1",8);
	
	
	
	
	public Vista(Modelo m)
	{
		modelo=m;
		frame = new JFrame();
		rx = new JTextArea();
		rx.setEditable(false);
		rx.setLineWrap(true);
		
		  rx = new JTextArea();
	        DefaultCaret caret = (DefaultCaret) rx.getCaret();
	        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);
		
		JScrollPane scrollRx = new JScrollPane(rx);
		//scrollRx.setAutoscrolls(true);
		//rx.setAutoscrolls(true);
		//scrollRx.set
		//scrollRx.setSize(500, 500);
		northSouthPanel = new JPanel(new BorderLayout());
		northSouthPanel.add(BorderLayout.CENTER,scrollRx);
		
		rssiBaliza0 = new JButton("Obtener RSSI");
		rssiBaliza1 = new JButton("Obtener RSSI de B2");
		
		rssiBaliza1.setVisible(false);
		limpiarDatos = new JButton("Limpiar gráfica");
	
		botones = new JPanel();

		botones.add(rssiBaliza0);
		botones.add(rssiBaliza1);

		distInput = new JTextField(8);
		
		JPanel distInputPanel = new JPanel();
		JLabel distLabel =new JLabel("Distancia de la muestra (cm): ");
		
		distInputPanel.add(distLabel);
		distInputPanel.add(distInput);
		distInputPanel.add(limpiarDatos);
		//botones.add(BoxLayout.Y_AXIS,distInputPanel);
		
		ejeXpanel= new JPanel();
		ejeXfijo= new JCheckBox("Eje X fijo");
		JLabel rangoX =new JLabel("Rango X: ");
		xLower= new MyTextField(0,8);
		xUpper= new MyTextField(1,8);
		ejeXpanel.add(ejeXfijo);
		ejeXpanel.add(rangoX);
		ejeXpanel.add(xLower);
		ejeXpanel.add(xUpper);
		
		ejeYpanel= new JPanel();
		ejeYfijo= new JCheckBox("Eje Y fijo");
		JLabel rangoY =new JLabel("Rango Y: ");
		yLower= new MyTextField(2,8);
		yUpper= new MyTextField(3,8);
		ejeYpanel.add(ejeYfijo);
		ejeYpanel.add(rangoY);
		ejeYpanel.add(yLower);
		ejeYpanel.add(yUpper);
		
		capturaContinua=new JCheckBox("Captura continua");
			

		nmuestrasPanel = new JPanel(new GridBagLayout());
		JPanel nmuestrasSubPanel = new JPanel();
		nmuestrasSubPanel.setLayout(new BoxLayout(nmuestrasSubPanel, BoxLayout.X_AXIS));
		JLabel numMuestrasLabel =new JLabel("Cantidad de muestras por punto:  ");
		nmuestrasSubPanel.add(numMuestrasLabel);
		nmuestrasSubPanel.add(numMuestras);
		nmuestrasPanel.add(nmuestrasSubPanel,
				new GridBagConstraints(0,0,1,1,0,0,
						GridBagConstraints.CENTER,
						GridBagConstraints.CENTER,
						new Insets (0,0,0,0),0,0));
	
		
		
		southPanel = new JPanel();
		southPanel.setLayout(new BoxLayout(southPanel, BoxLayout.Y_AXIS)); 
		southPanel.add(distInputPanel);
		southPanel.add(ejeXpanel);
		southPanel.add(ejeYpanel);
		southPanel.add(nmuestrasPanel);
		southPanel.add(capturaContinua);
		southPanel.add(botones);
		
		
		northSouthPanel.add(BorderLayout.SOUTH,southPanel);
		
		
		frame.getContentPane().add(BorderLayout.CENTER,northSouthPanel);

		
		grafica();
		
	}
	
	public void grafica()
	{
	
		dataset.addSeries(rssiIda);
		dataset.addSeries(rssiVuelta);
		/*dataset.addSeries(series2);
		dataset.addSeries(series3);*/
		
		


		// create the chart...
		chart = ChartFactory.createXYLineChart(
				"RSSI", // chart title
				"", // x axis label
				"RSSI (dB)", // y axis label
				dataset, // data
				PlotOrientation.VERTICAL,
				false, // include legend
				true, // tooltips
				false // urls
				);

		// NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
		chart.setBackgroundPaint(Color.white);

		// get a reference to the plot for further customisation...
		XYPlot plot = (XYPlot) chart.getPlot();
		
		plot.setBackgroundPaint(Color.lightGray);
		plot.setAxisOffset(new RectangleInsets(5.0, 5.0, 5.0, 5.0));
		plot.setDomainGridlinePaint(Color.white);
		plot.setRangeGridlinePaint(Color.white);
		XYLineAndShapeRenderer renderer
		= (XYLineAndShapeRenderer) plot.getRenderer();
		renderer.setBaseItemLabelGenerator(new StandardXYItemLabelGenerator());
		//renderer.setShapesFilled(true);
		// change the auto tick unit selection to integer units only...
		NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
		//rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
		// OPTIONAL CUSTOMISATION COMPLETED.
		ValueAxis rangeX = plot.getDomainAxis();

		rangeAxis.setAutoRangeIncludesZero(false);
		
		//rangeAxis.setRange(0, 12);
		//rangeX.setRange(0,12);

		datasetCont.addSeries(rssiIdaCont);
		datasetCont.addSeries(rssiVueltaCont);
		
		rssiIdaCont.setMaximumItemAge(30000);
		rssiVueltaCont.setMaximumItemAge(30000);

		
	}
	public void show()
	{
		
		chartFrame = new ChartFrame("Gr�ficos",chart);
		chartFrame.pack();
		chartFrame.setVisible(true);
		chartFrame.setDefaultCloseOperation(ChartFrame.DO_NOTHING_ON_CLOSE);
		frame.pack();
		frame.setSize(frame.getWidth(),500);
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}
	
	public void addController(Controlador c)
	{

		frame.addWindowListener(c.getPcWifiUnoWindowListener());	
		rssiBaliza0.addActionListener(c.getRssi0ButtonListener());
		rssiBaliza1.addActionListener(c.getRssi1ButtonListener());
		distInput.addKeyListener(c.getDistInputListener());
		ejeYfijo.addActionListener(c.getEjeYfijoActionListener());
		ejeXfijo.addActionListener(c.getEjeXfijoActionListener());
		xLower.addKeyListener(c.getRangeInputListener());
		xUpper.addKeyListener(c.getRangeInputListener());
		yLower.addKeyListener(c.getRangeInputListener());
		yUpper.addKeyListener(c.getRangeInputListener());
		capturaContinua.addActionListener(c.getCaptContListener());
		limpiarDatos.addActionListener(c.getLimpiarGraficaButtonListener());
		numMuestras.addKeyListener(c.getNumMuestrasInputListener());
	}

	@Override
	public void updateRxObserver(String s) {
		// TODO Auto-generated method stub
		rx.append(s+'\n');
	}

	@Override
	public void updateRssiBaliza1Button(boolean activo) {
		// TODO Auto-generated method stub
		rssiBaliza0.setEnabled(activo);
	}

	@Override
	public void updateRssiBaliza0Button(boolean activo) {
		// TODO Auto-generated method stub
		rssiBaliza1.setEnabled(activo);
	}

	@Override
	public void addMuestra(float x, float yIda, float yVuelta) {
		// TODO Auto-generated method stub
		if(modelo.getCapturaContinua())
		{
			Millisecond s = new Millisecond();
			
			rssiIdaCont.addOrUpdate(s,yIda);
			//rssiVueltaCont.addOrUpdate(s,yVuelta);
		}
		else
		{
			rssiIda.addOrUpdate(x,yIda);
			//rssiVuelta.addOrUpdate(x,yVuelta);
		}
	}

	@Override
	public void limpiarGrafica() {
		// TODO Auto-generated method stub
		rssiIda.clear();
		rssiVuelta.clear();
		
	}


	@Override
	public void cambiarModo() {
		// TODO Auto-generated method stub
		
		boolean cc = modelo.getCapturaContinua();
		capturaContinua.setSelected(cc);
		
		ejeXfijo.setEnabled(!cc);
		//ejeYfijo.setEnabled(!cc);
		distInput.setEnabled(!cc);
		xLower.setEnabled(!cc);
		xUpper.setEnabled(!cc);
		//yLower.setEnabled(!cc);
		//yUpper.setEnabled(!cc);
		rssiBaliza0.setEnabled(!cc);
		rssiBaliza1.setEnabled(!cc); 
		limpiarDatos.setEnabled(!cc);
		//delUltMuestra;
		
		XYPlot plot = (XYPlot) chart.getPlot();
		XYLineAndShapeRenderer renderer
		= (XYLineAndShapeRenderer) plot.getRenderer();
		if(cc)
		{
			rssiIdaCont.clear();
			rssiVueltaCont.clear();
			plot.setDataset(datasetCont);
			NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
			ValueAxis rangeX = plot.getDomainAxis();
			//rangeAxis.setAutoRange(true);
			//rangeAxis.setRange(-95, -50);
			rangeX.setAutoRange(true);
			
			ValueAxis axis =plot.getDomainAxis();
			axis.setLabel("");
			
			renderer.setShapesVisible(false);
		
			renderer.setBaseItemLabelsVisible(false);
			

		}
		else
		{
			//rssiIda.clear();
			//rssiVuelta.clear();
			plot.setDataset(dataset);
			updateEjeY();
			updateEjeX();
			
			ValueAxis axis =plot.getDomainAxis();
			axis.setLabel("Distancia (cm)");
			
			renderer.setShapesVisible(true);
			renderer.setBaseItemLabelsVisible(true);
		}
			
	}

	@Override
	public void updateEjeY() {
		// TODO Auto-generated method stub
		XYPlot plot = (XYPlot) chart.getPlot();
		NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
		
		
		if(modelo.getEjeYfijo())
		{
			float lower=modelo.getEjeYlower();
			float upper=modelo.getEjeYupper();
			System.out.println("("+lower+","+upper+")");
			if(lower<upper)
			{
				rangeAxis.setRange(lower,upper);
				System.out.println("Rango actualizado");
			}
			yLower.setEnabled(true);
			yUpper.setEnabled(true);
		}
		else
		{
			rangeAxis.setAutoRange(true);
			yLower.setEnabled(false);
			yUpper.setEnabled(false);
		}
	}

	@Override
	public void updateEjeX() {
		// TODO Auto-generated method stub
		XYPlot plot = (XYPlot) chart.getPlot();
		ValueAxis rangeX = plot.getDomainAxis();
		
		if(modelo.getEjeXfijo())
		{
			float lower=modelo.getEjeXlower();
			float upper=modelo.getEjeXupper();
			System.out.println("("+lower+","+upper+")");
			if(lower<upper)
				rangeX.setRange(lower,upper);
			xLower.setEnabled(true);
			xUpper.setEnabled(true);
		}
		else
		{
			rangeX.setAutoRange(true);
			xLower.setEnabled(false);
			xUpper.setEnabled(false);
		}
	}

	@Override
	public void inicializarVista() {
		// TODO Auto-generated method stub
		ejeXfijo.setSelected(modelo.getEjeXfijo());
		ejeYfijo.setSelected(modelo.getEjeYfijo());
		xLower.setText(""+modelo.getEjeXlower());
		xUpper.setText(""+modelo.getEjeXupper());
		xLower.setEnabled(modelo.getEjeXfijo());
		xUpper.setEnabled(modelo.getEjeXfijo());
		yLower.setText(""+modelo.getEjeYlower());
		yUpper.setText(""+modelo.getEjeYupper());
		yLower.setEnabled(modelo.getEjeYfijo());
		yUpper.setEnabled(modelo.getEjeYfijo());
		distInput.setText(""+modelo.getDistancia());
		capturaContinua.setSelected(modelo.getCapturaContinua());
		
	}


}
