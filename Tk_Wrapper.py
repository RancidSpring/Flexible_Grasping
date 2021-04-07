from tkinter import *
from PIL import ImageTk, Image
import trimesh
import os
import copy

class Object:
    def __init__(self, idx, way, scale=None):
        if scale is None:
            scale = [1, 1, 1]
        self.position = "basic"
        self.idx = idx
        self.upgrade_coef = 3
        self.way = way
        self.mesh_size = 0
        self.subdivision_coeff = 0
        self.decimate_coeff = 0
        self.scale = scale

class TkWrapper:
    def __init__(self):
        self.cube_way = "objects/objects/cursed_cube.obj"
        self.fls_way = "objects/objects/Flashlight.obj"
        self.bulb_way = "objects/objects/bulb.obj"
        self.root = Tk()
        self.root.title("Algorithm's GUI startup window")
        self.objects_num = 0
        self.subdivision_coeff = 0
        self.object_arr = []
        self.decimate_coeff = 0
        self.cube_id = 2
        self.fls_id = 3
        self.bulb_id = 6
        self.face_mode = 3
        self.Cube = Object(self.cube_id, self.cube_way, [0.04, 0.04, 0.04])
        self.Flashlight = Object(self.fls_id, self.fls_way, [0.03, 0.03, 0.03])
        self.Bulb = Object(self.bulb_id, self.bulb_way, [0.05, 0.05, 0.05])
        self.find_mesh_size()
        self.current_obj = self.Cube

    # def myClick1(self, entry):
    #     try:
    #         self.objects_num = int(entry.get())
    #         entry.delete(0, END)
    #
    #         for label in self.root.grid_slaves():
    #             if int(label.grid_info()["row"]) == 3 and int(label.grid_info()["column"]) == 1:
    #                 label.grid_forget()
    #
    #         myLabel = Label(self.root, text="The amount is:     " + str(self.objects_num), fg="green")
    #         myLabel.grid(row=3, column=1)
    #     except:
    #         for label in self.root.grid_slaves():
    #             if int(label.grid_info()["row"]) == 3 and int(label.grid_info()["column"]) == 1:
    #                 label.grid_forget()
    #         entry.delete(0, END)
    #         myLabel = Label(self.root, text="The input is not an integer", fg="red")
    #         myLabel.grid(row=3, column=1)


    def find_mesh_size(self):
        self.cube_len = len(trimesh.exchange.load.load(self.cube_way).faces)
        self.fls_len = len(trimesh.exchange.load.load(self.fls_way).faces)
        self.bulb_len = len(trimesh.exchange.load.load(self.bulb_way).faces)
        self.Cube.mesh_size = self.cube_len
        self.Flashlight.mesh_size = self.fls_len
        self.Bulb.mesh_size = self.bulb_len

    def myClick2(self, entry1, entry2, entry3):
        try:
            self.object_arr = []
            cube_num = int(entry1.get())
            fls_num = int(entry2.get())
            bulbs_num = int(entry3.get())

            # entry1.delete(0, END)
            # entry2.delete(0, END)
            # entry3.delete(0, END)

            for label in self.root.grid_slaves():
                if int(label.grid_info()["row"]) == 5 and int(label.grid_info()["column"]) == 1:
                    label.grid_forget()

            for i in range(cube_num):
                self.object_arr.append(self.cube_id)

            for i in range(fls_num):
                self.object_arr.append(self.fls_id)

            for i in range(bulbs_num):
                self.object_arr.append(self.bulb_id)

            myLabel = Label(self.root, text="Object array is successfully created, length: "+str(len(self.object_arr)), fg="green")
            myLabel.grid(row=5, column=1)
        except:
            entry1.delete(0, END)
            entry2.delete(0, END)
            entry3.delete(0, END)

            for label in self.root.grid_slaves():
                if int(label.grid_info()["row"]) == 5 and int(label.grid_info()["column"]) == 1:
                    label.grid_forget()

            myLabel = Label(self.root, text="The input is not an integer", fg="red")
            myLabel.grid(row=5, column=1)

    def valuecheck(self, value):
        face_num = self.current_obj.mesh_size
        valuelist = [face_num, face_num * 4, face_num * (4 ** 2), face_num * (4 ** 3)]
        newvalue = min(valuelist, key=lambda x: abs(x - float(value)))
        self.horizontal.set(newvalue)
        return

    def increase_faces(self, new_num):
        for label in self.root.grid_slaves():
            if int(label.grid_info()["row"]) == 12 and int(label.grid_info()["column"]) == 1:
                label.grid_forget()
        self.current_obj.subdivision_coeff = new_num
        myLabel = Label(self.root, text="Number of faces increased 4**"+str(self.current_obj.subdivision_coeff)+" times", fg="green")
        myLabel.grid(row=12, column=1)

    def reduce_per_cent(self, new_num):
        for label in self.root.grid_slaves():
            if int(label.grid_info()["row"]) == 12 and int(label.grid_info()["column"]) == 1:
                label.grid_forget()
        self.current_obj.decimate_coeff = 1 - (new_num / self.current_obj.mesh_size)
        myLabel = Label(self.root, text=str(self.current_obj.decimate_coeff) + " per cent is reduced", fg="green")
        myLabel.grid(row=12, column=1)

    def leave_the_mesh(self):
        myLabel = Label(self.root, text="The number of faces is preserved", fg="green")
        myLabel.grid(row=12, column=1)

    def facemode(self, value):
        for label in self.root.grid_slaves():
            if 12 >= int(label.grid_info()["row"]) >= 10 and int(label.grid_info()["column"]) >= 1:
                label.grid_forget()
        mylabel = Label(self.root, text="The face mode is chosen", fg="green").grid(row=12, column=1)
        self.current_obj.upgrade_coef = value
        if self.current_obj.upgrade_coef == 1:
            value = IntVar()
            face_num = self.current_obj.mesh_size
            valuelist = [face_num, face_num * 4, face_num * (4 ** 2), face_num * (4 ** 3)]
            self.horizontal = Scale(self.root, from_=min(valuelist), to=max(valuelist), variable=value, command=self.valuecheck, orient="horizontal")
            self.horizontal.grid(row=10, column=1)
            myButton2 = Button(self.root, text="Confirm", command=lambda: self.increase_faces(valuelist.index(value.get())), padx=50, pady=5)
            myButton2.grid(row=10, column=2)
        elif self.current_obj.upgrade_coef == 2:
            value = IntVar()
            horizontal = Scale(self.root, from_=self.current_obj.mesh_size, to=0, orient="horizontal", variable=value)
            horizontal.grid(row=10, column=1)
            horizontal.set(self.current_obj.mesh_size)
            myButton2 = Button(self.root, text="Confirm", command=lambda: self.reduce_per_cent(value.get()), padx=50, pady=5)
            myButton2.grid(row=10, column=2)
        elif self.current_obj.upgrade_coef == 3:
            for label in self.root.grid_slaves():
                if 12 >= int(label.grid_info()["row"]) >= 10 and int(label.grid_info()["column"]) >= 1:
                    label.grid_forget()
            myButton2 = Button(self.root, text="Confirm", command=lambda: self.leave_the_mesh(), padx=50, pady=5)
            myButton2.grid(row=10, column=2)

    def change_window_drop(self, clicked):
        for label in self.root.grid_slaves():
            if 12 >= int(label.grid_info()["row"]) > 8 and int(label.grid_info()["column"]) >= 0:
                label.grid_forget()
        if clicked.get() == "Cube":
            self.current_obj = self.Cube
        elif clicked.get() == "Flashlight":
            self.current_obj = self.Flashlight
        elif clicked.get() == "Light Bulb":
            self.current_obj = self.Bulb

        mylabel1 = Label(self.root, text=str(clicked.get())+" currently consists of "+str(self.current_obj.mesh_size)+" faces").grid(row=8, column=1)

        r = IntVar()
        Radiobutton(self.root, text="Increase the number of faces", variable=r, value=1, command=lambda: self.facemode(r.get())).grid(row=9, column=0)
        Radiobutton(self.root, text="Reduce the number of faces", variable=r, value=2, command=lambda: self.facemode(r.get())).grid(row=10, column=0)
        Radiobutton(self.root, text="Leave the mesh as untouched", variable=r, value=3, command=lambda: self.facemode(r.get())).grid(row=11, column=0)

    def run_meshlab(self):
        # os.system("/snap/meshlab/36/AppRun")
        os.system("~/blender/blender-2.83.4-linux64/blender")

    def start_the_program(self):
        self.object_load = []
        for i in self.object_arr:
            if i == self.cube_id:
                self.object_load.append(copy.deepcopy(self.Cube))
            if i == self.fls_id:
                self.object_load.append(copy.deepcopy(self.Flashlight))
            if i == self.bulb_id:
                self.object_load.append(copy.deepcopy(self.Bulb))

        self.root.destroy()
        print("zhopa")

    def main_func(self):
        self.root.option_add('*Font', 'Times 19')

        # entry_label = Label(self.root, text="Welcome to Flexible Grasp Algorithm")
        # entry_label.grid(row=0, column=1)
        #
        # self.root.option_add('*Font', 'Times 12')


        # FIRST STEP
        lbl2 = Label(self.root, text="1) How many objects of each type to be loaded:")
        lbl2.grid(row=0, column=1)

        img1 = ImageTk.PhotoImage(Image.open("photos/cursed_cube.png").resize((200, 150)))
        img2 = ImageTk.PhotoImage(Image.open("photos/flashlight.png").resize((200, 150)))
        img3 = ImageTk.PhotoImage(Image.open("photos/bulb.png").resize((200, 150)))

        lblim1 = Label(self.root, image=img1).grid(row=1, column=0)
        lblim2 = Label(self.root, image=img2).grid(row=2, column=0)
        lblim3 = Label(self.root, image=img3).grid(row=3, column=0)

        lbl3 = Label(self.root, text="Cube").grid(row=1, column=1)
        lbl4 = Label(self.root, text="Flashlight").grid(row=2, column=1)
        lbl5 = Label(self.root, text="Light Bulb").grid(row=3, column=1)

        # lbl6 = Label(self.root, text="Cube").grid(row=4, column=1)
        # lbl7 = Label(self.root, text="Flashlight").grid(row=5, column=1)
        # lbl8 = Label(self.root, text="Light Bulb").grid(row=6, column=1)
        #
        # lbl9 = Label(self.root, text="Cube").grid(row=4, column=1)
        # lbl10 = Label(self.root, text="Flashlight").grid(row=5, column=1)
        # lbl11 = Label(self.root, text="Light Bulb").grid(row=6, column=1)

        e2 = Entry(self.root, width=10)
        e2.grid(row=1, column=2)
        e2.insert(0, "0")
        e3 = Entry(self.root, width=10)
        e3.grid(row=2, column=2)
        e3.insert(0, "0")
        e4 = Entry(self.root, width=10)
        e4.grid(row=3, column=2)
        e4.insert(0, "0")

        myButton2 = Button(self.root, text="Confirm", command=lambda: self.myClick2(e2, e3, e4), padx=50, pady=5)
        myButton2.grid(row=4, column=1)

        # SECOND STEP
        lbl3 = Label(self.root, text="2) Would you like to change the mesh size?")
        lbl3.grid(row=6, column=1)

        clicked = StringVar()
        clicked.set("Cube")
        drop1 = OptionMenu(self.root, clicked, "Cube", "Flashlight", "Light Bulb")
        drop1.grid(row=7, column=1)

        myButton3 = Button(self.root, text="Confirm", command=lambda: self.change_window_drop(clicked), padx=50, pady=5)
        myButton3.grid(row=7, column=2)

        # THIRD STEP

        lbl3 = Label(self.root, text="3) Additional Mesh Upgrade")
        lbl3.grid(row=13, column=1)

        myButton3 = Button(self.root, text="Confirm", command=lambda: self.run_meshlab(), padx=50, pady=5)
        myButton3.grid(row=14, column=1)

        # FINAL STEP

        Button(self.root, text="Startup the Program", command=self.start_the_program).grid(row=15, column=1)

        self.root.mainloop()


if __name__ == "__main__":

    tk = TkWrapper()
    tk.main_func()
