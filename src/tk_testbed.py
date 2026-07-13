import tkinter as tk

root = tk.Tk()
root.title("Menubutton Example")
root.geometry("300x200")

# 1. Create the Menubutton container
menu_btn = tk.Menubutton(root, text="Select Options", relief="raised")
menu_btn.pack(pady=50)

# 2. Create the Menu child element inside the Menubutton
dropdown_menu = tk.Menu(menu_btn, tearoff=0)
dropdown_menu.add_command(label="Profile", command=lambda: print("Profile Clicked"))
dropdown_menu.add_command(label="Settings", command=lambda: print("Settings Clicked"))

# 3. Explicitly link the Menu back to the Menubutton
menu_btn["menu"] = dropdown_menu

root.mainloop()