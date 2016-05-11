class Drive:
    
    def __init__(self, lf_module, lr_module, rf_module, rr_module):
        self.lf_module = lf_module
        self.lr_module = lr_module
        self.rf_module = rf_module
        self.rr_module = rr_module
        
        self.modules = [self.lf_module,
                  self.lr_module,
                  self.rf_module,
                  self.rr_module
                  ]
        
    
    def on_enable(self):
        for module in self.modules:
            module.zero()
        
    
        