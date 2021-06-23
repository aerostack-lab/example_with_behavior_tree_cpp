#include "query_belief.h"

QueryBelief::QueryBelief(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, {})
{
  this->query=query;
  
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("query_belief", query_belief, "query_belief");
  query_belief_srv = n.serviceClient<aerostack_msgs::QueryBelief>('/' + drone_id_namespace + '/' + query_belief);

}


BT::NodeStatus QueryBelief::tick()
{
  std::string query_expr;
  if ( !getInput<std::string>("query", query_expr))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  query=query_expr;

  std::cout << "QueryBelief: " << this->name() << std::endl; 
  bool activation_result=askQueryBelief();
  /*If activation_result=true the service was succesfully completed otherwise it was a failure completion*/
  if(activation_result)
  {
    //setColor(COLOR_GREEN);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    //setColor(COLOR_RED);
    return BT::NodeStatus::FAILURE;
  }
}

bool QueryBelief::askQueryBelief()
{
  //variable obtention
  std::string belief_expression=query;
  std::vector< std::pair<char,double> > variables_pair=stringToPair(belief_expression);
  //for loop, we create pair <var_name,-1>  
  for(int i=0 ; i<variables_pair.size() ; i++)
  {
    variables->insert(variables_pair[i]);
  }
  //service call
  req_activate.query=query;
  query_belief_srv.call(req_activate,res_activate);
  bool service_success=res_activate.success;
  if(service_success)
  {
    std::string query_result;
    query_result=res_activate.substitutions;
    //for loop, we update the value of the variables with the pair <var_name,substitution_value>
    std::vector< std::pair<char,double> > variables_match=parserResponse(query_result);
    for(int i=0 ; i<variables_match.size() ; i++)
    {
      variables->setValue(variables_match[i]);
    }
  }
  
  return service_success;
}

std::vector< std::pair<char,double> > QueryBelief::stringToPair(std::string str)
{
  std::vector<std::pair<char,double>> res;
  for(int i=0; i<str.size();i++)
  {
    // initial case i.e. position(?R,(?X,?Y,?Z))
    if(str[i]=='?')
    {
      std::pair <char,double> variable (str[i+1],-1);
      res.push_back(variable);
       
    }
    //substitution case  i.e. {R:0,X:3,Y:4,Z:6}
    if(str[i]==':')
    {
      //revisar esto
      std::pair <char,double> variable (str[i-1],std::stod(str[i+1]+" "));
      res.push_back(variable);
    }
  }
  //return vector pair
  return res;
}

std::vector< std::pair<char,double> > QueryBelief::parserResponse(std::string str)
{
  std::vector<std::pair<char,double>> res;
  
  std::string parsed;
  std::stringstream input_stringstream(str);

  while(std::getline(input_stringstream,parsed,'\n'))
  {
    char nombre_var=parsed[0];
    std::string valor_var = parsed.substr(2, parsed.find("\n"));
    for(int i=0;i<valor_var.size();i++)
    {
      if(valor_var[i]=='.')
      {
        valor_var[i]=',';
      }
    }
    std::pair <char,double> variable (nombre_var,std::stod(valor_var));
    res.push_back(variable);
  }
  new_variable=true;
  //return vector pair
  return res;
}