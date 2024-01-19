import tkinter as tk
import serial

# Utworzenie połączenia z portem szeregowym
ser = serial.Serial('COM5', 115200)

# Globalne zmienne do przechowywania ostatnich ustawień PID
last_kp = 0.1
last_ki = 0.2
last_kd = 0.1

def send_rpm():
    try:
        rpm_value = int(rpm_entry.get())
        if 2000 <= rpm_value <= 15000:
            command = f"SET:{rpm_value}:{last_kp}:{last_ki}:{last_kd}\n"
            ser.write(command.encode())
            status_label.config(text=f"Wysłano: RPM={rpm_value}")
        else:
            status_label.config(text="Wartość RPM poza zakresem 2000-15000!")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawną wartość liczbową dla RPM")

def send_pid():
    global last_kp, last_ki, last_kd
    try:
        last_kp = float(kp_entry.get())
        last_ki = float(ki_entry.get())
        last_kd = float(kd_entry.get())
        # Tutaj możesz zaimplementować logikę wysyłania wartości PID
        # jeżeli jest taka potrzeba, na przykład:
        # command = f"SET_PID:{last_kp}:{last_ki}:{last_kd}\n"
        # ser.write(command.encode())
        status_label.config(text=f"Wysłano: Kp={last_kp}, Ki={last_ki}, Kd={last_kd}")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawne wartości liczbowe dla PID")

root = tk.Tk()
root.title("RPM i Regulator PID")

# Ramka dla RPM
frame_rpm = tk.Frame(root)
frame_rpm.pack(padx=10, pady=10)

tk.Label(frame_rpm, text="RPM (2000-15000):").pack(side=tk.LEFT)
rpm_entry = tk.Entry(frame_rpm)
rpm_entry.pack(side=tk.LEFT)

tk.Button(frame_rpm, text="Potwierdź RPM", command=send_rpm).pack(side=tk.LEFT)

# Ramka dla PID
frame_pid = tk.Frame(root)
frame_pid.pack(padx=10, pady=10)

tk.Label(frame_pid, text="Kp:").pack(side=tk.LEFT)
kp_entry = tk.Entry(frame_pid)
kp_entry.insert(0, "0.1")  # Ustawienie domyślnej wartości Kp
kp_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Ki:").pack(side=tk.LEFT)
ki_entry = tk.Entry(frame_pid)
ki_entry.insert(0, "0.2")  # Ustawienie domyślnej wartości Ki
ki_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Kd:").pack(side=tk.LEFT)
kd_entry = tk.Entry(frame_pid)
kd_entry.insert(0, "0.1")  # Ustawienie domyślnej wartości Kd
kd_entry.pack(side=tk.LEFT)

# Przycisk do wysyłania parametrów PID
tk.Button(root, text="Wyślij Parametry PID", command=send_pid).pack()

# Etykieta wyświetlająca status
status_label = tk.Label(root, text="Status: Nie wysłano danych")
status_label.pack(pady=10)

root.mainloop()