using System;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace WroVisionDebugger;

internal sealed class MainForm : Form
{
    private readonly ComboBox _portSelector = new();
    private readonly ComboBox _baudSelector = new();
    private readonly Button _refreshButton = new();
    private readonly Button _connectButton = new();
    private readonly CheckBox _flashCheckBox = new();
    private readonly TrackBar _flashBrightness = new();
    private readonly Label _flashBrightnessLabel = new();
    private readonly PictureBox _originalPicture = CreatePictureBox();
    private readonly PictureBox _correctedPicture = CreatePictureBox();
    private readonly Label _connectionLabel = new();
    private readonly Label _frameLabel = new();
    private readonly Label _statisticsLabel = new();
    private readonly DataGridView _cellTable = new();
    private readonly DebugPacketParser _parser = new();
    private readonly object _serialLock = new();

    private SerialPort? _serialPort;
    private CancellationTokenSource? _readerCancellation;
    private Task? _readerTask;
    private Bitmap? _originalBitmap;
    private Bitmap? _correctedBitmap;
    private long _lastByteCount;
    private DateTime _lastStatisticsUpdate = DateTime.UtcNow;
    private uint _nextCommandSequence = 1;

    public MainForm()
    {
        Text = "WRO Vision Debugger";
        Width = 1400;
        Height = 900;
        MinimumSize = new Size(1000, 700);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Color.FromArgb(28, 30, 34);
        ForeColor = Color.White;

        _parser.PacketReceived += OnPacketReceived;
        BuildInterface();
        RefreshPorts();
        SetConnectionState(false, "Desconectado");
    }

    protected override void OnFormClosed(FormClosedEventArgs e)
    {
        Disconnect();
        _originalBitmap?.Dispose();
        _correctedBitmap?.Dispose();
        base.OnFormClosed(e);
    }

    private void BuildInterface()
    {
        TableLayoutPanel root = new()
        {
            Dock = DockStyle.Fill,
            RowCount = 3,
            ColumnCount = 1,
            Padding = new Padding(10)
        };
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 68));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 32));
        Controls.Add(root);

        FlowLayoutPanel toolbar = new()
        {
            Dock = DockStyle.Fill,
            FlowDirection = FlowDirection.LeftToRight,
            WrapContents = false,
            Padding = new Padding(0, 5, 0, 0)
        };
        root.Controls.Add(toolbar, 0, 0);

        toolbar.Controls.Add(CreateLabel("Puerto:"));
        ConfigureComboBox(_portSelector, 120);
        toolbar.Controls.Add(_portSelector);

        toolbar.Controls.Add(CreateLabel("Velocidad:"));
        ConfigureComboBox(_baudSelector, 100);
        _baudSelector.Items.AddRange(new object[] { "115200", "460800", "921600" });
        _baudSelector.SelectedItem = "921600";
        toolbar.Controls.Add(_baudSelector);

        ConfigureButton(_refreshButton, "Actualizar");
        _refreshButton.Click += (_, _) => RefreshPorts();
        toolbar.Controls.Add(_refreshButton);

        ConfigureButton(_connectButton, "Conectar");
        _connectButton.Click += (_, _) => ToggleConnection();
        toolbar.Controls.Add(_connectButton);

        _flashCheckBox.Text = "Flash cámara";
        _flashCheckBox.AutoSize = true;
        _flashCheckBox.ForeColor = Color.White;
        _flashCheckBox.BackColor = BackColor;
        _flashCheckBox.Margin = new Padding(15, 7, 0, 0);
        _flashCheckBox.CheckedChanged += (_, _) =>
            SendFlashCommand(_flashCheckBox.Checked
                ? (byte)_flashBrightness.Value
                : (byte)0);
        toolbar.Controls.Add(_flashCheckBox);

        toolbar.Controls.Add(CreateLabel("Brillo:"));
        _flashBrightness.Minimum = 0;
        _flashBrightness.Maximum = 255;
        _flashBrightness.Value = 255;
        _flashBrightness.TickFrequency = 25;
        _flashBrightness.Width = 130;
        _flashBrightness.Height = 30;
        _flashBrightness.ValueChanged += (_, _) =>
        {
            _flashBrightnessLabel.Text = $"{_flashBrightness.Value * 100 / 255}%";
            if (_flashCheckBox.Checked)
            {
                SendFlashCommand((byte)_flashBrightness.Value);
            }
        };
        toolbar.Controls.Add(_flashBrightness);

        _flashBrightnessLabel.AutoSize = true;
        _flashBrightnessLabel.Padding = new Padding(0, 7, 8, 0);
        _flashBrightnessLabel.Text = "100%";
        toolbar.Controls.Add(_flashBrightnessLabel);

        _connectionLabel.AutoSize = true;
        _connectionLabel.Padding = new Padding(15, 7, 0, 0);
        toolbar.Controls.Add(_connectionLabel);

        _frameLabel.AutoSize = true;
        _frameLabel.Padding = new Padding(20, 7, 0, 0);
        toolbar.Controls.Add(_frameLabel);

        TableLayoutPanel imageLayout = new()
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = Color.Black,
            Padding = new Padding(2)
        };
        imageLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        imageLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        root.Controls.Add(imageLayout, 0, 1);
        imageLayout.Controls.Add(CreateImagePanel("ORIGINAL ANOTADA", _originalPicture), 0, 0);
        imageLayout.Controls.Add(CreateImagePanel("PERSPECTIVA CORREGIDA", _correctedPicture), 1, 0);

        TableLayoutPanel bottomLayout = new()
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1
        };
        bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 75));
        bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25));
        root.Controls.Add(bottomLayout, 0, 2);

        ConfigureCellTable();
        bottomLayout.Controls.Add(_cellTable, 0, 0);

        _statisticsLabel.Dock = DockStyle.Fill;
        _statisticsLabel.Padding = new Padding(15, 10, 5, 5);
        _statisticsLabel.BackColor = Color.FromArgb(40, 43, 48);
        _statisticsLabel.TextAlign = ContentAlignment.TopLeft;
        _statisticsLabel.Font = new Font(FontFamily.GenericSansSerif, 10, FontStyle.Regular);
        bottomLayout.Controls.Add(_statisticsLabel, 1, 0);
    }

    private static Label CreateLabel(string text)
    {
        return new Label
        {
            Text = text,
            AutoSize = true,
            Padding = new Padding(8, 7, 3, 0)
        };
    }

    private static void ConfigureComboBox(ComboBox comboBox, int width)
    {
        comboBox.Width = width;
        comboBox.DropDownStyle = ComboBoxStyle.DropDownList;
    }

    private static void ConfigureButton(Button button, string text)
    {
        button.Text = text;
        button.AutoSize = true;
        button.Height = 30;
    }

    private static Panel CreateImagePanel(string title, PictureBox pictureBox)
    {
        Panel panel = new() { Dock = DockStyle.Fill, BackColor = Color.Black };
        Label label = new()
        {
            Text = title,
            Dock = DockStyle.Top,
            Height = 28,
            TextAlign = ContentAlignment.MiddleCenter,
            BackColor = Color.FromArgb(40, 43, 48),
            ForeColor = Color.White
        };
        panel.Controls.Add(pictureBox);
        panel.Controls.Add(label);
        return panel;
    }

    private static PictureBox CreatePictureBox()
    {
        return new PictureBox
        {
            Dock = DockStyle.Fill,
            BackColor = Color.Black,
            SizeMode = PictureBoxSizeMode.Zoom
        };
    }

    private void ConfigureCellTable()
    {
        _cellTable.Dock = DockStyle.Fill;
        _cellTable.AllowUserToAddRows = false;
        _cellTable.AllowUserToDeleteRows = false;
        _cellTable.AllowUserToResizeRows = false;
        _cellTable.ReadOnly = true;
        _cellTable.RowHeadersVisible = false;
        _cellTable.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
        _cellTable.BackgroundColor = Color.FromArgb(40, 43, 48);
        _cellTable.BorderStyle = BorderStyle.None;
        _cellTable.DefaultCellStyle.BackColor = Color.White;
        _cellTable.DefaultCellStyle.ForeColor = Color.Black;
        _cellTable.DefaultCellStyle.SelectionBackColor = Color.FromArgb(0, 120, 215);
        _cellTable.DefaultCellStyle.SelectionForeColor = Color.White;
        _cellTable.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(55, 58, 64);
        _cellTable.ColumnHeadersDefaultCellStyle.ForeColor = Color.White;
        _cellTable.EnableHeadersVisualStyles = false;

        _cellTable.Columns.Add("cell", "Casilla");
        _cellTable.Columns.Add("color", "Color");
        _cellTable.Columns.Add("confidence", "Confianza");
        _cellTable.Columns.Add("rgb", "RGB");
        _cellTable.Columns.Add("hsv", "HSV");

        for (int index = 0; index < 12; index++)
        {
            _cellTable.Rows.Add(index + 1, "Esperando", "-", "-", "-");
        }
    }

    private void RefreshPorts()
    {
        string? selectedPort = _portSelector.SelectedItem as string;
        string[] ports = SerialPort.GetPortNames();
        Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
        _portSelector.Items.Clear();
        _portSelector.Items.AddRange(ports);

        if (selectedPort is not null && Array.Exists(ports, port => port == selectedPort))
        {
            _portSelector.SelectedItem = selectedPort;
        }
        else if (ports.Length > 0)
        {
            _portSelector.SelectedIndex = 0;
        }
    }

    private void ToggleConnection()
    {
        if (_serialPort is null)
        {
            Connect();
        }
        else
        {
            Disconnect();
        }
    }

    private void Connect()
    {
        if (_portSelector.SelectedItem is not string portName ||
            !int.TryParse(_baudSelector.SelectedItem as string, out int baudRate))
        {
            SetConnectionState(false, "Selecciona puerto y velocidad");
            return;
        }

        try
        {
            SerialPort serialPort = new(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 100,
                WriteTimeout = 1000,
                DtrEnable = false,
                RtsEnable = false
            };
            serialPort.Open();

            lock (_serialLock)
            {
                _serialPort = serialPort;
                _readerCancellation = new CancellationTokenSource();
                _readerTask = Task.Run(() => ReadSerial(serialPort, _readerCancellation.Token));
            }

            SendFlashCommand(_flashCheckBox.Checked
                ? (byte)_flashBrightness.Value
                : (byte)0);
            _connectButton.Text = "Desconectar";
            SetConnectionState(true, $"Conectado a {portName}");
        }
        catch (Exception exception)
        {
            SetConnectionState(false, $"Error: {exception.Message}");
        }
    }

    private void Disconnect()
    {
        SerialPort? serialPort;
        CancellationTokenSource? cancellation;
        lock (_serialLock)
        {
            serialPort = _serialPort;
            cancellation = _readerCancellation;
            _serialPort = null;
            _readerCancellation = null;
        }

        cancellation?.Cancel();
        if (serialPort is not null)
        {
            try
            {
                serialPort.Close();
                serialPort.Dispose();
            }
            catch (IOException)
            {
            }
            catch (InvalidOperationException)
            {
            }
        }

        _connectButton.Text = "Conectar";
        SetConnectionState(false, "Desconectado");
    }

    private void ReadSerial(SerialPort serialPort, CancellationToken cancellationToken)
    {
        byte[] buffer = new byte[8192];
        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                int count = serialPort.Read(buffer, 0, buffer.Length);
                if (count > 0)
                {
                    _parser.Feed(buffer.AsSpan(0, count));
                }
            }
            catch (TimeoutException)
            {
            }
            catch (InvalidOperationException)
            {
                break;
            }
            catch (IOException)
            {
                break;
            }
        }
    }

    private void SendFlashCommand(byte brightness)
    {
        byte[] packet = DebugCommandCodec.CreateFlashCommand(
            _nextCommandSequence++, brightness);
        lock (_serialLock)
        {
            if (_serialPort is null || !_serialPort.IsOpen)
            {
                return;
            }

            try
            {
                _serialPort.Write(packet, 0, packet.Length);
            }
            catch (InvalidOperationException)
            {
                SetConnectionState(false, "No se pudo enviar el flash");
            }
            catch (IOException)
            {
                SetConnectionState(false, "No se pudo enviar el flash");
            }
        }
    }

    private void OnPacketReceived(DebugPacket packet)
    {
        if (IsDisposed)
        {
            return;
        }

        BeginInvoke(() =>
        {
            switch (packet.Type)
            {
                case DebugPacketType.OriginalJpeg:
                    UpdateImage(ref _originalBitmap, _originalPicture, packet.Payload);
                    break;
                case DebugPacketType.CorrectedJpeg:
                    UpdateImage(ref _correctedBitmap, _correctedPicture, packet.Payload);
                    break;
                case DebugPacketType.Metadata:
                    UpdateMetadata(packet);
                    break;
            }

            _frameLabel.Text = $"Frame: {packet.Sequence}";
            UpdateStatisticsLabel();
        });
    }

    private void UpdateImage(ref Bitmap? target, PictureBox pictureBox, byte[] jpeg)
    {
        try
        {
            using MemoryStream stream = new(jpeg, writable: false);
            using Image decoded = Image.FromStream(stream, useEmbeddedColorManagement: false, validateImageData: true);
            Bitmap replacement = new(decoded);
            Bitmap? previous = target;
            target = replacement;
            pictureBox.Image = replacement;
            previous?.Dispose();
        }
        catch (ArgumentException)
        {
            _parser.RegisterFormatError();
        }
    }

    private void UpdateMetadata(DebugPacket packet)
    {
        if (!MetadataCodec.TryDecode(packet.Payload, out GridMetadata metadata))
        {
            _parser.RegisterFormatError();
            return;
        }

        for (int index = 0; index < metadata.Cells.Length; index++)
        {
            CellMetadata cell = metadata.Cells[index];
            _cellTable.Rows[index].Cells[1].Value = MetadataCodec.ColorName(cell.ColorCode);
            _cellTable.Rows[index].Cells[2].Value = $"{cell.Confidence}%";
            _cellTable.Rows[index].Cells[3].Value = $"{cell.Red}, {cell.Green}, {cell.Blue}";
            _cellTable.Rows[index].Cells[4].Value = $"{cell.Hue} / {cell.Saturation} / {cell.Value}";
        }

        _statisticsLabel.Text =
            $"Rejilla: {(metadata.GridValid ? "VALIDA" : "NO DETECTADA")}\n" +
            $"Confianza global: {metadata.OverallConfidence}%\n" +
            $"Marco negro: {metadata.BorderScore}%\n" +
            $"Estructura 4x3: {metadata.GridScore}%\n" +
            $"Celdas validas: {metadata.ValidColorCells}/12\n" +
            $"Codigo rechazo: {metadata.RejectionCode}\n" +
            $"Procesamiento: {metadata.ProcessingMilliseconds} ms\n" +
            $"FPS ESP32: {metadata.FramesPerSecondTenths / 10.0:F1}\n" +
            $"Paquetes: {_parser.PacketsReceived}\n" +
            $"CRC erróneos: {_parser.CrcErrors}\n" +
            $"Errores de formato: {_parser.FormatErrors}";
    }

    private void UpdateStatisticsLabel()
    {
        DateTime now = DateTime.UtcNow;
        if ((now - _lastStatisticsUpdate).TotalMilliseconds < 500)
        {
            return;
        }

        long bytes = _parser.BytesReceived;
        long delta = bytes - _lastByteCount;
        _lastByteCount = bytes;
        _lastStatisticsUpdate = now;
        _statisticsLabel.Text =
            $"Bytes recibidos: {bytes}\n" +
            $"Tasa aproximada: {delta * 2 / 1024.0:F1} KB/s\n" +
            $"Paquetes: {_parser.PacketsReceived}\n" +
            $"CRC erróneos: {_parser.CrcErrors}\n" +
            $"Errores de formato: {_parser.FormatErrors}";
    }

    private void SetConnectionState(bool connected, string message)
    {
        _connectionLabel.Text = message;
        _connectionLabel.ForeColor = connected ? Color.LightGreen : Color.LightSalmon;
    }
}
